#include "leg_grid.h"
#include "kalmanfilter.h"

LegGrid::LegGrid(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
    n_(_n),
    tf_listener_(_tf_listener)
{
    ROS_INFO("Constructing an instance of Leg Grid.");
    init();
}

void LegGrid::init()
{
    initialized = false;

    double varU[2] = {0.1, 0.1}; // Motion model uncertainties
    double varZ[2] = {0.25, toRadian(20.0)}; // Measurement uncertainties

    /***
     * TODO: The uncertainty of range and angle should be a function of range
     */

    try
    {
        kalman_filter = new KalmanFilter(varU, varZ);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new kalman filter: bad_alloc caught: " << ba.what() << '\n';
    }

    try
    {
        tf_listener_ = new tf::TransformListener();
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Leg Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }


    CellProbability_t cp;
    cp.free = 0.1;
    cp.human = 0.9;
    cp.unknown = 0.5;

    SensorFOV_t sfov;
    sfov.range.max = 10;
    sfov.range.min = 0.5;
    sfov.angle.max = toRadian(270);
    sfov.angle.min = toRadian(-270);

    try
    {
        grid = new Grid(40, sfov, 0.5, cp, 0.9, 0.1);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }

    predicted_leg_pub_ = n_.advertise<geometry_msgs::PoseArray>("predicted_legs",10);

}

void LegGrid::syncCallBack(const geometry_msgs::PoseArrayConstPtr &leg_msg,
                           const nav_msgs::OdometryConstPtr &encoder_msg)
{
    ros::Time now = ros::Time::now();
    grid->crtsn_array.current.poses.clear();
    /***
     * Transform detected legs from laser frame to base_footprint frame
     */

    if(!transformToBase(leg_msg, grid->crtsn_array.current, false)){

        ROS_WARN("Can not transform from laser to base_footprint");

    } else{

        /*** Make Sure that all of detected legs are transformed and stored in grid Cartesian array. ***/

        ROS_ASSERT(leg_msg->poses.size() == grid->crtsn_array.current.poses.size());

        /*** Convert Cartesian points to Polar points ***/
        grid->getPose(grid->crtsn_array.current);

        ROS_ASSERT(grid->polar_array.current.size() == leg_msg->poses.size());

    }

    /***
     * For predicting the human target, we assume that robot is stationary
     * and human is moving with the robot's opposite direction.
     * To do so, the opposite of robot velocity read from encoder data
     *  has set as detected legs's velocity
     */

    leg_velocity_.linear = - sqrt( pow(encoder_msg->twist.twist.linear.x,2) + pow(encoder_msg->twist.twist.linear.y,2) );
    leg_velocity_.angular = - encoder_msg->twist.twist.angular.z;

    diff_time_ = now - last_time_;
    last_time_ = now;
}

bool LegGrid::transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                                             geometry_msgs::PoseArray& target,
                                             bool debug)
{
    bool can_transform = true;

    geometry_msgs::PointStamped source_point, target_point;
    geometry_msgs::Pose target_pose;

    target_point.header = target.header;
    source_point.header = source->header;

    for(size_t i = 0; i < source->poses.size(); i++){

        source_point.point = source->poses.at(i).position;

        try
        {
            tf_listener_->transformPoint("/base_footprint", source_point, target_point);

            if (debug) {

                tf::StampedTransform _t;
                tf_listener_->lookupTransform("base_footprint", source_point.header.frame_id, ros::Time(0), _t);

                ROS_INFO("From %s to bfp: [%.2f, %.2f, %.2f] (%.2f %.2f %.2f %.2f)",
                         source_point.header.frame_id.c_str(),
                         _t.getOrigin().getX(), _t.getOrigin().getY(), _t.getOrigin().getZ(),
                         _t.getRotation().getX(), _t.getRotation().getY(), _t.getRotation().getZ(),
                         _t.getRotation().getW());

            }

            can_transform = true;

        } catch(tf::TransformException& ex)
        {

            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_footprint\": %s",
                      source_point.header.frame_id.c_str(), ex.what());

            can_transform = false;

        }

        target_pose.position = target_point.point;
        target_pose.position.z = 0.0;
        target.poses.push_back(target_pose);

    }

    return can_transform;
}


void LegGrid::makeStates()
{

    /***
     * measurement = recieved leg detector data
     * control command = leg velocity (-robot velocity)
     * last state = last estimated state
     * estimated state = fusion of last state and current measurement
     */

    std::vector<PolarPose> last_state(grid->polar_array.past);
    std::vector<PolarPose> measurement(grid->polar_array.current);

    current_measurement.clear();
    current_state.clear();
    available_measurement.clear();

    PolarPose x;
    PolarPose y;

    /***
     * Go through last states. If there is a close measurement (match) for last state,
     * pass that as its measurement. If not set a zero measurement for that last state.
     *
     * Go through measurement, if it is not already match with a state, add it to current
     * measurement with zero state
     */

    for(uint8_t i = 0; i < last_state.size(); i++){

        x = last_state.at(i);
        current_state.push_back(x);
        bool match = false;

        for(uint8_t j = 0 ; j < measurement.size(); j++){

            y = measurement.at(j);

            if (x.distance(y.range, y.angle) < 0.5){

                match = true;
                available_measurement.push_back(true);
                current_measurement.push_back(y);
                measurement.at(j).range = 1e+5;
                j = measurement.size();

            }
        }

        if(!match){

            y.range = y.angle = 0.0;
            current_measurement.push_back(y);
            available_measurement.push_back(false);
        }
    }

    ROS_INFO("current: %lu  past: %lu", current_state.size(), last_state.size());

    ROS_ASSERT(current_measurement.size() == current_state.size());
    ROS_ASSERT(available_measurement.size() == current_state.size());
    ROS_ASSERT(current_state.size() == last_state.size());

    for (uint8_t i = 0; i < measurement.size(); i++){

        y = measurement.at(i);

        if(y.range < 1e+3){

            current_measurement.push_back(y);
            available_measurement.push_back(true);
            x = y;
            x.var_range = x.var_angle = 0.0;
            current_state.push_back(x);

        }
    }

    ROS_ASSERT(current_measurement.size() == current_state.size());
    ROS_ASSERT(available_measurement.size() == current_state.size());

}


void LegGrid::spin()
{
    grid->polar_array.predicted.clear();

    makeStates();

    double X_old[2], Z[2];
    double P_old[4] = {0.0, 0.0, 0.0, 0.0};
    double U[2] = {leg_velocity_.linear, leg_velocity_.angular};

    PolarPose x;

    for(uint8_t i = 0; i < current_state.size(); i++){

        X_old[0] = current_state.at(i).range;
        X_old[1] = current_state.at(i).angle;
        P_old[0] = current_state.at(i).var_range;
        P_old[3] = current_state.at(i).var_angle;

        Z[0] = current_measurement.at(i).range;
        Z[1] = current_measurement.at(i).angle;

        kalman_filter->predict(U, diff_time_.toSec(), X_old, P_old);

        if(available_measurement.at(i))
            kalman_filter->update(Z);

        x.range = kalman_filter->X_new[0];
        x.angle = kalman_filter->X_new[1];
        x.var_range = kalman_filter->P_new[0];
        x.var_angle = kalman_filter->P_new[3];

        if(x.var_angle < 1.0 && x.var_range < 1.0)
            grid->polar_array.predicted.push_back(x);

    }

    grid->polar_array.past.clear();
    grid->polar_array.past = grid->polar_array.predicted;

    grid->polar2Crtsn(grid->polar_array.predicted, grid->crtsn_array.predicted);
    predicted_leg_pub_.publish(grid->crtsn_array.predicted);

}

LegGrid::~LegGrid()
{
    ROS_INFO("Deconstructing the constructed LegGrid.");
    delete grid;
    delete kalman_filter;
    delete tf_listener_;
}
