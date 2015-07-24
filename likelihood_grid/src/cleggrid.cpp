#include "cleggrid.h"
#define DEBUG false
#define BASE_FOOTPRINT_FRAME false

CLegGrid::CLegGrid(ros::NodeHandle _n, tf::TransformListener *_tf_listener, int _probability_projection_step):
    n_(_n),
    tf_listener_(_tf_listener),
    KFTracker_(2, 2, 2),
    probability_projection_step(_probability_projection_step)
{
    ROS_INFO("Constructing an instance of Leg Grid.");
    init();
}

void CLegGrid::initKF()
{
    float varU[2] = {0.1, (float) angles::from_degrees(1.0)}; // Motion (process) uncertainties
    float varZ[2] = {0.1,(float)  angles::from_degrees(1.0)}; // Measurement uncertainties
    setIdentity(KFTracker_.transitionMatrix);

    KFTracker_.processNoiseCov = *(cv::Mat_<float>(2, 2) << varU[0], 0.0,  0.0, varU[1]);
    KFTracker_.measurementNoiseCov = *(cv::Mat_<float>(2,2) << varZ[0], 0.0,   0.0, varZ[1]);
    KFTracker_.statePre = *(cv::Mat_<float>(2,1) << 0.0, 0.0);

    setIdentity(KFTracker_.measurementMatrix);
    setIdentity(KFTracker_.controlMatrix);
    setIdentity(KFTracker_.errorCovPre, cv::Scalar::all(0.1));
    setIdentity(KFTracker_.errorCovPost, cv::Scalar::all(0.0));
    setIdentity(KFTracker_.gain, cv::Scalar::all(0.0));
}

void CLegGrid::initTfListener()
{
    try
    {
        tf_listener_ = new tf::TransformListener();
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Leg Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }
}

void CLegGrid::initGrid()
{
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
        grid_ = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1, probability_projection_step);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
    prob_.poses.resize(grid_->grid_size);

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.x = grid_->map.cell.at(i).cartesian.x;
        prob_.poses.at(i).position.y = grid_->map.cell.at(i).cartesian.y;
        prob_.poses.at(i).position.z = 0.0;
    }
}

void CLegGrid::init()
{
    last_time_ = ros::Time::now();
    last_seen_leg_ = ros::Time::now();

    initKF();
    initTfListener();
    initGrid();

    predicted_leg_pub_ = n_.advertise<geometry_msgs::PoseArray>("predicted_legs",10);
    grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("leg/grid",10);
    prob_pub_ = n_.advertise<geometry_msgs::PoseArray>("leg/probability",10);
    proj_pub_ = n_.advertise<geometry_msgs::PoseArray>("leg/projection",10);
}

void CLegGrid::callbackClear()
{
    if(!grid_->polar_array.current.empty())
        grid_->polar_array.current.clear();

}
void CLegGrid::computeObjectVelocity()
{
    /*
     * For predicting the human target, we assume that robot is stationary
     * and human is moving with the robot's opposite direction.
     * To do so, the opposite of robot velocity read from encoder data
     *  has set as detected legs's velocity
     */

    velocity_.linear = - sqrt( pow(encoder_reading_.twist.twist.linear.x,2) + pow(encoder_reading_.twist.twist.linear.y,2) );
    velocity_.angular = - encoder_reading_.twist.twist.angular.z;
}

void CLegGrid::encoderCallBack(const nav_msgs::OdometryConstPtr &encoder_msg)
{
    encoder_reading_.twist = encoder_msg->twist;
    computeObjectVelocity();
}

void CLegGrid::legsCallBack(const geometry_msgs::PoseArrayConstPtr &leg_msg)

//void CLegGrid::syncCallBack(const geometry_msgs::PoseArrayConstPtr &leg_msg,
//                           const nav_msgs::OdometryConstPtr &encoder_msg)
{
    callbackClear();

    ros::Time now = ros::Time::now();

    diff_time_ = now - last_time_;
    last_time_ = now;

    ros::Duration d = now - last_seen_leg_;

    base_footprint_legs_.poses.clear();

    if(! BASE_FOOTPRINT_FRAME)
    {
        if(!transformToBase(leg_msg, base_footprint_legs_, false)) /* Transform detected legs from laser frame to base_footprint frame */
        {
            ROS_WARN("Can not transform from laser to base_footprint");
            return;
        }
    }
    else
    {
        base_footprint_legs_.poses.assign(leg_msg->poses.begin(), leg_msg->poses.end());
    }

    ROS_ASSERT(leg_msg->poses.size() == base_footprint_legs_.poses.size());

    if(base_footprint_legs_.poses.empty())
    {
        if(d.toSec() < 1.0)
        {
            keepLastLegs();
            return;
        }
    }
    else
    {
        last_seen_leg_ = now;
        filterLegs();
        grid_->getPose(filtered_legs_);
    }
}

void CLegGrid::keepLastLegs()
{
    PolarPose p;
    if(! grid_->polar_array.current.empty()) grid_->polar_array.current.clear();

    for(size_t i = 0; i < grid_->polar_array.past.size() ; i++)
    {
        ROS_INFO_COND(DEBUG, "Keeping last heard sound source.");

        p.range = grid_->polar_array.past.at(i).range + velocity_.linear * diff_time_.toSec();
        p.angle = grid_->polar_array.past.at(i).angle + velocity_.angular * diff_time_.toSec();
        grid_->polar_array.current.push_back(p);
    }
}
void CLegGrid::filterLegs()
{

    geometry_msgs::Pose new_leg, p1, p2;

    /* check for close detected legs */
    if(base_footprint_legs_.poses.size() < 2)
    {
        filtered_legs_.poses = base_footprint_legs_.poses;
    }
    else
    {
        if(!filtered_legs_.poses.empty()) filtered_legs_.poses.clear();
        filtered_legs_.poses.push_back(base_footprint_legs_.poses.at(0));

        for(uint8_t i = 1; i < base_footprint_legs_.poses.size(); i++)
        {
            p1.position = base_footprint_legs_.poses.at(i).position;
            p2.position = filtered_legs_.poses.back().position;

            if(sqrt(p1.position.x*p1.position.x + p1.position.y*p1.position.y) > 10.0) continue;

            if(pointDistance(p1.position, p2.position) < 1.0)
            {
                filtered_legs_.poses.pop_back();
                new_leg.position.x = (p1.position.x + p2.position.x) / 2;
                new_leg.position.y = (p1.position.y + p2.position.y) / 2;
                filtered_legs_.poses.push_back(new_leg);

            } else
            {
                filtered_legs_.poses.push_back(p1);
            }

        }
    }
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


void CLegGrid::clearStates()
{
    if(!meas_.empty()) meas_.clear();
    if(!cmeas_.empty()) cmeas_.clear();
    if(!cstate_.empty()) cstate_.clear();
    if(!match_meas_.empty()) match_meas_.clear();

}

void CLegGrid::addLastStates()
{
    std::vector<PolarPose> lstate(grid_->polar_array.past);

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
        cstate_.push_back(lstate.at(i));

        if(meas_.empty())
        {
            PolarPose z(-1.0, -1.0);
            cmeas_.push_back(z); /* Only for matching number of current states with current measurements*/
        }

        /* Look for the closest current measurement to the last state*/
        else
        {
            std::vector<float> dist;

            for(size_t j = 0 ; j < meas_.size(); j++)
            {
                dist.push_back(lstate.at(i).distance(meas_.at(j)));
            }

            it.at(i) = (std::min_element(dist.begin(), dist.end()));

            uint8_t it2 = it.at(i) - dist.begin();

            if(*it.at(i) < 1.0)
            {
                cmeas_.push_back(meas_.at(it2));
                match_meas_.at(it2) = true;
            }
            else
            {
                PolarPose z(-1.0, -1.0);
                cmeas_.push_back(z);
            }
        }
    }

    /* number of current measurements and current states and last states
       should be the same at this level*/

    ROS_ASSERT(cmeas_.size() == cstate_.size());
    ROS_ASSERT(cstate_.size() == lstate.size());
}

void CLegGrid::addMeasurements()
{
    /* Go through measurements, if it is not already matched with a state, add it to current
     * measurement. corresponding current state will be the same as current measurement with
       zero variances.*/
    for (size_t i = 0; i < meas_.size(); i++)
    {
        if(!match_meas_.at(i))
        {
            cmeas_.push_back(meas_.at(i));
            meas_.at(i).setZeroVar();
            cstate_.push_back(meas_.at(i));
        }
    }
}

void CLegGrid::filterStates()
{

    ROS_ASSERT(cmeas_.size() == cstate_.size());

    std::vector<PolarPose> fstate, fmeas;
    PolarPose nstate, s1, s2, nmeas, m1, m2;

    /* check for close states */
    if(cstate_.size() < 2)
    {
        fstate = cstate_;
        fmeas = cmeas_;
    }
    else
    {
        if(!fstate.empty()) fstate.clear();
        fstate.push_back(cstate_.at(0));

        if(!fmeas.empty()) fmeas.clear();
        fmeas.push_back(cmeas_.at(0));

        for(uint8_t i = 1; i < cstate_.size(); i++)
        {
            s1 = cstate_.at(i);
            s2 = fstate.back();

            m1 = cmeas_.at(i);
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

    if(!cstate_.empty()) cstate_.clear();
    cstate_ = fstate;

    if(!cmeas_.empty()) cmeas_.clear();
    cmeas_ = fmeas;
}

void CLegGrid::makeStates()
{

    /*
     * measurement = recieved leg detector data
     * control command = leg velocity (-robot velocity)
     * last state = last estimated state
     */
    clearStates();

    ROS_INFO_COND(DEBUG,"size of new leg msgs: %lu", grid_->polar_array.current.size());

    if(diff_time_.toSec() < 1.0)
    {
        meas_.assign(grid_->polar_array.current.begin(), grid_->polar_array.current.end());
        if(!grid_->polar_array.current.empty())
            grid_->polar_array.current.clear();
    }

    match_meas_.resize(meas_.size(), false);

    addLastStates();
    addMeasurements();
    filterStates();

    ROS_ASSERT(cmeas_.size() == cstate_.size());
}

void CLegGrid::updateKF()
{
    PolarPose x;

    cv::Mat statePast = *(cv::Mat_<float>(2, 1) << 0.0, 0.0);

    cv::Mat errorCovPast = *(cv::Mat_<float>(2, 2) << 1e-3, 0.0,0.0, 1e-3);

    cv::Mat control = *(cv::Mat_<float>(2, 1) << 0.0, 0.0);


    for(uint8_t i = 0; i < cstate_.size(); i++){


        statePast.at<float>(0,0) = cstate_.at(i).range;
        statePast.at<float>(1,0) = angles::normalize_angle(cstate_.at(i).angle);

        errorCovPast.at<float>(0,0) = cstate_.at(i).var_range;
        errorCovPast.at<float>(1,1)= cstate_.at(i).var_angle;

        control.at<float>(0,0) = velocity_.linear * diff_time_.toSec();
        control.at<float>(1,0) = velocity_.angular * diff_time_.toSec();

        /*** Prediction ***/
        KFTracker_.statePre = statePast + control;
        KFTracker_.errorCovPre = errorCovPast + KFTracker_.processNoiseCov;


        /*** Correction ***/
        if(cmeas_.at(i).range > 0.0)
        {
            KFmeasurement_ = *(cv::Mat_<float>(2, 1) << cmeas_.at(i).range,
                              (float) angles::normalize_angle(cmeas_.at(i).angle));

            cv::Mat temp1 =  *(cv::Mat_<float>(2, 2) << 0.0, 0.0, 0.0, 0.0);
            cv::Mat temp2 =  *(cv::Mat_<float>(2, 2) << 1.0, 0.0, 0.0, 1.0);

            temp1 = (KFTracker_.measurementMatrix * KFTracker_.errorCovPre * KFTracker_.measurementMatrix.t()) +
                    KFTracker_.measurementNoiseCov;

            KFTracker_.gain = KFTracker_.errorCovPre * KFTracker_.measurementMatrix.t() * temp1.inv();

            KFTracker_.statePost = KFTracker_.statePre + KFTracker_.gain *
                    (KFmeasurement_ - KFTracker_.measurementMatrix * KFTracker_.statePre);

            KFTracker_.errorCovPost = (temp2 - (KFTracker_.gain * KFTracker_.measurementMatrix)) * KFTracker_.errorCovPre;
        }

        else
        {
            KFTracker_.statePost = KFTracker_.statePre;
            KFTracker_.errorCovPost = KFTracker_.errorCovPre;
        }

        x.range = KFTracker_.statePost.at<float>(0, 0);
        x.angle = KFTracker_.statePost.at<float>(1, 0);

        x.var_range = KFTracker_.errorCovPost.at<float>(0, 0);
        x.var_angle = KFTracker_.errorCovPost.at<float>(1, 1);

        ROS_ASSERT(x.var_range > 0.0 && x.var_angle > 0.0);

        if(x.var_angle < (float) angles::from_degrees(5.0) && x.var_range < 0.5)
        {
            grid_->polar_array.predicted.push_back(x);
        }
    }
}

void CLegGrid::publishProbability()
{
    float max = -1000;
    if(prob_.poses.empty()) ROS_WARN("there is no probability");

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.z = grid_->posterior.at(i);
        max = std::max(grid_->posterior.at(i), max);
    }

    prob_.header.stamp = ros::Time::now();
    if(prob_pub_.getNumSubscribers() > 0)
        prob_pub_.publish(prob_);
}

void CLegGrid::publishOccupancyGrid()
{
    if(grid_->occupancy_grid.data.empty()) ROS_WARN("there is no probability");
    grid_->occupancy_grid.header.stamp = ros::Time::now();
    grid_->occupancy_grid.info.map_load_time = ros::Time::now();
    if(grid_pub_.getNumSubscribers() > 0)
        grid_pub_.publish(grid_->occupancy_grid);
}

void CLegGrid::publishPredictedLegs()
{
    grid_->polar2Crtsn(grid_->polar_array.predicted, grid_->crtsn_array.predicted);
    grid_->crtsn_array.predicted.header.stamp = ros::Time::now();

    if(predicted_leg_pub_.getNumSubscribers() > 0)
        predicted_leg_pub_.publish(grid_->crtsn_array.predicted);
}

void CLegGrid::publishProjection()
{
    if(proj_pub_.getNumSubscribers() > 0)
        proj_pub_.publish(grid_->grid_projection);
}

void CLegGrid::spin()
{
    if(!grid_->polar_array.predicted.empty()) grid_->polar_array.predicted.clear();

    ROS_INFO_COND(DEBUG,"--- spin ---");

    makeStates();
    updateKF();
    grid_->updateGrid(1);
    publishPredictedLegs();
    publishProbability();
    publishOccupancyGrid();
    grid_->projectGrid();
    publishProjection();
}

CLegGrid::~CLegGrid()
{
    ROS_INFO("Deconstructing the constructed LegGrid.");
    delete grid_;
    delete tf_listener_;
}
