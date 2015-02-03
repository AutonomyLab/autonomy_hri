#include "cleggrid.h"

CLegGrid::CLegGrid(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
    n_(_n),
    tf_listener_(_tf_listener),
    KFTracker(2, 2, 2)
{
    ROS_INFO("Constructing an instance of Leg Grid.");
    init();
}

void CLegGrid::init()
{
    float varU[2] = {0.01, (float) angles::from_degrees(1.0)}; // Motion (process) uncertainties
    float varZ[2] = {0.1,(float)  angles::from_degrees(0.5)}; // Measurement uncertainties

    setIdentity(KFTracker.transitionMatrix);

    KFTracker.processNoiseCov = *(cv::Mat_<float>(2, 2) << varU[0], 0.0,  0.0, varU[1]);
    KFTracker.measurementNoiseCov = *(cv::Mat_<float>(2,2) << varZ[0], 0.0,   0.0, varZ[1]);

    setIdentity(KFTracker.measurementMatrix);
    setIdentity(KFTracker.controlMatrix);
    setIdentity(KFTracker.errorCovPre, cv::Scalar::all(0.1));
    setIdentity(KFTracker.errorCovPost, cv::Scalar::all(0.0));
    setIdentity(KFTracker.gain, cv::Scalar::all(0.0));

    KFTracker.statePre.at<float>(0, 0) = 0.0;
    KFTracker.statePre.at<float>(1, 0) = 0.0;

    /*
     * TODO: The uncertainty of range and angle should be a function of range
     */

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
    sfov.angle.max = angles::from_degrees(135.0);
    sfov.angle.min = angles::from_degrees(-135.0);

    try
    {
        grid = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }

    predicted_leg_pub_ = n_.advertise<geometry_msgs::PoseArray>("predicted_legs",10);
    leg_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("leg_grid",10);


}

void CLegGrid::syncCallBack(const geometry_msgs::PoseArrayConstPtr &leg_msg,
                           const nav_msgs::OdometryConstPtr &encoder_msg)
{
    ros::Time now = ros::Time::now();
    if(!grid->polar_array.current.empty()) grid->polar_array.current.clear();
    geometry_msgs::PoseArray tmp_legs;

    /*
     * Transform detected legs from laser frame to base_footprint frame
     */

    if(!transformToBase(leg_msg, tmp_legs, false)){

        ROS_WARN("Can not transform from laser to base_footprint");

    } else{

        /* Make Sure that all of detected legs are transformed and stored in grid Cartesian array. */
            tmp_legs.poses = leg_msg->poses;

        ROS_ASSERT(leg_msg->poses.size() == tmp_legs.poses.size());

        geometry_msgs::PoseArray filter_legs;
        geometry_msgs::Pose new_leg, p1, p2;

        /* check for close detected legs */
        if(leg_msg->poses.size() < 2)
        {
            filter_legs.poses = leg_msg->poses;
        }
        else
        {
            if(!filter_legs.poses.empty()) filter_legs.poses.clear();
            filter_legs.poses.push_back(tmp_legs.poses.at(0));

            for(uint8_t i = 1; i < tmp_legs.poses.size(); i++)
            {
                p1.position = tmp_legs.poses.at(i).position;
                p2.position = filter_legs.poses.back().position;

                if(pointDistance(p1.position, p2.position) < 1.0)
                {
                    filter_legs.poses.pop_back();
                    new_leg.position.x = (p1.position.x + p2.position.x) / 2;
                    new_leg.position.y = (p1.position.y + p2.position.y) / 2;
                    filter_legs.poses.push_back(new_leg);

                } else
                {
                    filter_legs.poses.push_back(p1);
                }
            }
        }

        /* Convert Cartesian points to Polar points */
        grid->getPose(filter_legs);
    }

    /*
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

bool CLegGrid::transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                                             geometry_msgs::PoseArray& target,
                                             bool debug)
{
    bool can_transform = true;

    if(source->header.frame_id == "base_footprint") return can_transform;

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


void CLegGrid::makeStates()
{

    /*
     * measurement = recieved leg detector data
     * control command = leg velocity (-robot velocity)
     * last state = last estimated state
     */

    ROS_WARN("size of new leg msgs: %lu", grid->polar_array.current.size());

    std::vector<PolarPose> lstate(grid->polar_array.past);
    std::vector<PolarPose> meas;


    if(diff_time_.toSec() < 1.0)
    {
        meas.assign(grid->polar_array.current.begin(), grid->polar_array.current.end());
        if(!grid->polar_array.current.empty()) grid->polar_array.current.clear();
    }

    std::vector<bool> match_meas(meas.size(), false);
    if(!cmeas.empty()) cmeas.clear();
    if(!cstate.empty()) cstate.clear();

    /*
     *
     * Go through last states. If there is a close measurement (match) for last state,
     * pass that as its measurement i.e. we use nearest neighbour for data association
     * If there is no match, set a zero measurement for that state. This will increase
     * the uncertainity of that state during time. If the uncertainity is more than a
     * threshhold, we remove that state.
     *
     */

    /* Go through all of last tracked states*/
    for(size_t i = 0; i < lstate.size(); i++)
    {
        std::vector<std::vector<float>::const_iterator> it(lstate.size());

        /* Add last state to the current state */
        cstate.push_back(lstate.at(i));

        if(meas.empty())
        {
            PolarPose z(-1.0, -1.0);
            cmeas.push_back(z); /* Only for matching number of current states with current measurements*/
        }

        /* Look for the closest current measurement to the last state*/
        else
        {
            std::vector<float> dist;

            for(size_t j = 0 ; j < meas.size(); j++)
            {
                dist.push_back(lstate.at(i).distance(meas.at(j)));
            }

            it.at(i) = (std::min_element(dist.begin(), dist.end()));

            uint8_t it2 = it.at(i) - dist.begin();

            if(*it.at(i) < 1.0)
            {
                cmeas.push_back(meas.at(it2));
                match_meas.at(it2) = true;
            }
            else
            {
                PolarPose z(-1.0, -1.0);
                cmeas.push_back(z);
            }
        }
    }

    /* number of current measurements and current states and last states
       should be the same at this level*/
    ROS_ASSERT(cmeas.size() == cstate.size());
    ROS_ASSERT(cstate.size() == lstate.size());

    /* Go through measurements, if it is not already matched with a state, add it to current
     * measurement. corresponding current state will be the same as current measurement with
       zero variances.*/
    for (size_t i = 0; i < meas.size(); i++)
    {
        if(!match_meas.at(i))
        {
            cmeas.push_back(meas.at(i));
            meas.at(i).setZeroVar();
            cstate.push_back(meas.at(i));
        }
    }

    ROS_ASSERT(cmeas.size() == cstate.size());

    /****/
    std::vector<PolarPose> fstate, fmeas;
    PolarPose nstate, s1, s2, nmeas, m1, m2;

    /* check for close states */
    if(cstate.size() < 2)
    {
        fstate = cstate;
        fmeas = cmeas;
    }
    else
    {
        if(!fstate.empty()) fstate.clear();
        fstate.push_back(cstate.at(0));

        if(!fmeas.empty()) fmeas.clear();
        fmeas.push_back(cmeas.at(0));

        for(uint8_t i = 1; i < cstate.size(); i++)
        {
            s1 = cstate.at(i);
            s2 = fstate.back();

            m1 = cmeas.at(i);
            m2 = fmeas.back();

            if(s1.distance(s2) < 1.0)
            {
                fstate.pop_back();
                nstate.range = (s1.range + s2.range) / 2;
                nstate.angle = (s1.angle + s2.angle) / 2;
                nstate.setZeroVar();
                fstate.push_back(nstate);

                fmeas.pop_back();
                nmeas.range = (m1.range + m2.range) / 2;
                nmeas.angle = (m1.angle + m2.angle) / 2;
                fmeas.push_back(nstate);

            } else
            {
                fstate.push_back(s1);
                fmeas.push_back(m1);
            }
        }
    }

    if(!cstate.empty()) cstate.clear();
    cstate = fstate;

    if(!cmeas.empty()) cmeas.clear();
    cmeas = fmeas;
    ROS_ASSERT(cmeas.size() == cstate.size());
}


void CLegGrid::spin()
{
    if(!grid->polar_array.predicted.empty()) grid->polar_array.predicted.clear();

    ROS_INFO("--- spin ---");

    makeStates();

    PolarPose x;

    for(uint8_t i = 0; i < cstate.size(); i++){

        //TODO: Add these matrises to the kalman filter class
        cv::Mat statePast = *(cv::Mat_<float>(2, 1) << cstate.at(i).range,
                              angles::normalize_angle(cstate.at(i).angle));

        cv::Mat errorCovPast = *(cv::Mat_<float>(2, 2) << cstate.at(i).var_range, 0.0,
                                 0.0, cstate.at(i).var_angle);

        cv::Mat control = *(cv::Mat_<float>(2, 1) << leg_velocity_.linear * diff_time_.toSec(),
                              leg_velocity_.angular * diff_time_.toSec());

        /*** Prediction ***/
        KFTracker.statePre = statePast + control;
        KFTracker.errorCovPre = errorCovPast + KFTracker.processNoiseCov;


        /*** Correction ***/
        if(cmeas.at(i).range > 0.0)
        {
            KFmeasurement = *(cv::Mat_<float>(2, 1) << cmeas.at(i).range,
                              (float) angles::normalize_angle(cmeas.at(i).angle));

            cv::Mat temp1 =  *(cv::Mat_<float>(2, 2) << 0.0, 0.0, 0.0, 0.0);
            cv::Mat temp2 =  *(cv::Mat_<float>(2, 2) << 1.0, 0.0, 0.0, 1.0);

            temp1 = (KFTracker.measurementMatrix * KFTracker.errorCovPre * KFTracker.measurementMatrix.t()) +
                    KFTracker.measurementNoiseCov;

            KFTracker.gain = KFTracker.errorCovPre * KFTracker.measurementMatrix.t() * temp1.inv();

            KFTracker.statePost = KFTracker.statePre + KFTracker.gain *
                    (KFmeasurement - KFTracker.measurementMatrix * KFTracker.statePre);

            KFTracker.errorCovPost = (temp2 - (KFTracker.gain * KFTracker.measurementMatrix)) * KFTracker.errorCovPre;
        }

        else
        {
            KFTracker.statePost = KFTracker.statePre;
            KFTracker.errorCovPost = KFTracker.errorCovPre;
        }

        x.range = KFTracker.statePost.at<float>(0, 0);
        x.angle = KFTracker.statePost.at<float>(1, 0);

        x.var_range = KFTracker.errorCovPost.at<float>(0, 0);
        x.var_angle = KFTracker.errorCovPost.at<float>(1, 1);

        ROS_ASSERT(x.var_range > 0.0 && x.var_angle > 0.0);

        if(x.var_angle < (float) angles::from_degrees(10.0) && x.var_range < 0.5)
        {
            grid->polar_array.predicted.push_back(x);
        }
    }

    if(!grid->polar_array.past.empty()) grid->polar_array.past.clear();
    grid->polar_array.past = grid->polar_array.predicted;

    grid->updateGrid();

    /*** publish predicted legs and occupancy grid ***/
    grid->polar2Crtsn(grid->polar_array.predicted, grid->crtsn_array.predicted);
    predicted_leg_pub_.publish(grid->crtsn_array.predicted);
    leg_grid_pub_.publish(grid->occupancy_grid);

}

CLegGrid::~CLegGrid()
{
    ROS_INFO("Deconstructing the constructed LegGrid.");
    delete grid;
    delete tf_listener_;
}
