#include "csoundgrid.h"

#define DEBUG false

CSoundGrid::CSoundGrid()
{
}


CSoundGrid::CSoundGrid(ros::NodeHandle _n, tf::TransformListener *_tf_listener):
    n_(_n),
    tf_listener_(_tf_listener),
    KFTracker_(2, 2, 2)
{
    ROS_INFO("Constructing an instance of Sound Grid.");
    init();
}

void CSoundGrid::init()
{
    last_heard_sound_ = ros::Time::now() - ros::Duration(1000.0);
    last_time_ = ros::Time::now() - ros::Duration(1000.0);

    initKF();
    initTfListener();
    initGrid();

    grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("sound_grid",10);
    prob_pub_ = n_.advertise<geometry_msgs::PoseArray>("sound_probability", 10);
}


void CSoundGrid::initKF()
{
    float varU[2] = {7.0, (float) angles::from_degrees(1.0)}; // Motion (process) uncertainties
    float varZ[2] = {7.0, (float) angles::from_degrees(1.0)}; // Measurement uncertainties


    KFTracker_.processNoiseCov = *(cv::Mat_<float>(2, 2) << varU[0], 0.0,  0.0, varU[1]);
    KFTracker_.measurementNoiseCov = *(cv::Mat_<float>(2,2) << varZ[0], 0.0,   0.0, varZ[1]);
    KFTracker_.statePre = *(cv::Mat_<float>(2,1) << 0.0, 0.0);

    setIdentity(KFTracker_.measurementMatrix);
    setIdentity(KFTracker_.controlMatrix);
    setIdentity(KFTracker_.transitionMatrix);
    setIdentity(KFTracker_.errorCovPre, cv::Scalar::all(0.1));
    setIdentity(KFTracker_.errorCovPost, cv::Scalar::all(0.0));
    setIdentity(KFTracker_.gain, cv::Scalar::all(0.0));
}


void CSoundGrid::initTfListener()
{
    try
    {
        tf_listener_ = new tf::TransformListener();
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "In Sound Grid Interface Constructor: bad_alloc caught: " << ba.what() << '\n';
    }
}


void CSoundGrid::initGrid()
{
    CellProbability_t cp;
    cp.free = 0.1;
    cp.human = 0.9;
    cp.unknown = 0.5;

    SensorFOV_t sfov;
    sfov.range.max = 10.0;
    sfov.range.min = 0.5;
    sfov.angle.max = angles::from_degrees(90.0);
    sfov.angle.min = angles::from_degrees(-90.0);

    try
    {
        grid_ = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new SoundGrid: bad_alloc caught: " << ba.what() << '\n';
    }

    prob_.poses.resize(grid_->grid_size);

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.x = grid_->map.cell.at(i).cartesian.x;
        prob_.poses.at(i).position.y = grid_->map.cell.at(i).cartesian.y;
        prob_.poses.at(i).position.z = 0.0;
    }
}


void CSoundGrid::callbackClear()
{
    if(!grid_->polar_array.current.empty())
        grid_->polar_array.current.clear();

    if(!ss_reading_.empty())
        ss_reading_.clear();
}


void CSoundGrid::rejectNotValidSoundSources(float p)
{

    for(size_t i = 0; i < ss_reading_.size(); i++)
    {
        if(fabs(ss_reading_.at(i).y) < 1e-9 || ss_reading_.at(i).power < p)
            ss_reading_.erase(ss_reading_.begin() + i);
        ROS_INFO_COND(DEBUG,"Rejecting not valid sound sources.");
    }
}



void CSoundGrid::syncCallBack(const hark_msgs::HarkSourceConstPtr &sound_msg,
                              const nav_msgs::OdometryConstPtr &encoder_msg)
{
    ROS_INFO_COND(DEBUG,"Received encoder and sound source msgs");
    callbackClear();

    /**** ENCODER ****/

    ros::Time now = ros::Time::now();
    encoder_reading_.twist = encoder_msg->twist;
    computeObjectVelocity();

    diff_time_ = now - last_time_;
    last_time_ = now;

    /**** SOUND ****/

    ros::Duration d = now - last_heard_sound_;
    ss_reading_.assign(sound_msg->src.begin(), sound_msg->src.end());
    rejectNotValidSoundSources(25.0);

    if(sound_msg->src.empty())
    {
        if(d.toSec() < 5.0 )
            keepLastSound();
        return;
    }

    else
    {
        last_heard_sound_ = now;
        addSoundSource();
    }
}


void CSoundGrid::addSoundSource()
{
    polar_ss_.range = 6.0;

    for(size_t i = 0; i < ss_reading_.size(); i++)
    {
        polar_ss_.angle = angles::from_degrees(ss_reading_.at(i).azimuth);
        grid_->polar_array.current.push_back(polar_ss_);

        ROS_INFO_COND(DEBUG,"Sound source direction: %f", polar_ss_.angle);

//        addMirrorSoundSource();
    }
}


void CSoundGrid::addMirrorSoundSource()
{
    int8_t sign = (polar_ss_.angle >= 0.0) ? 1 : -1;
    polar_ss_.angle = (sign * M_PI) - polar_ss_.angle;
    grid_->polar_array.current.push_back(polar_ss_);
    ROS_INFO_COND(DEBUG,"Sound source direction: %f", polar_ss_.angle);
}


void CSoundGrid::keepLastSound()
{
    if(!grid_->polar_array.past.empty())
    {
        ROS_INFO_COND(DEBUG, "Keeping last heard sound source.");
        grid_->polar_array.current.assign(grid_->polar_array.past.begin(),
                                         grid_->polar_array.past.end());
    }
}


void CSoundGrid::computeObjectVelocity()
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


void CSoundGrid::makeStates()
{
    /*
     * measurement = recieved leg detector data
     * control command = leg velocity (-robot velocity)
     * last state = last estimated state
     */

    clearStates();
    ROS_WARN("size of new sound msgs: %lu", grid_->polar_array.current.size());

    if(diff_time_.toSec() < 2.0)
    {
        meas_.assign(grid_->polar_array.current.begin(), grid_->polar_array.current.end());
        if(!grid_->polar_array.current.empty()) grid_->polar_array.current.clear();
    }

    match_meas_.resize(meas_.size(), false);

    addLastStates();
    addMeasurements();
    filterStates();

    ROS_ASSERT(cmeas_.size() == cstate_.size());
}



void CSoundGrid::addLastStates()
{

    /*
     * Go through last states. If there is a close measurement (match) for last state,
     * pass that as its measurement i.e. we use nearest neighbour for data association
     * If there is no match, set a zero measurement for that state. This will increase
     * the uncertainity of that state during time. If the uncertainity is more than a
     * threshhold, we remove that state.
     */
    std::vector<PolarPose> lstate(grid_->polar_array.past);
    for(size_t i = 0; i < lstate.size(); i++)
    {
        std::vector<std::vector<float>::const_iterator> it(lstate.size());
        cstate_.push_back(lstate.at(i));

        if(meas_.empty())
        {
            PolarPose z(-1.0, -1.0);
            cmeas_.push_back(z);
        }

        /* Look for the closest current measurement to the last state*/
        else
        {
            std::vector<float> dist;

            for(size_t j = 0 ; j < meas_.size(); j++)
            {
                dist.push_back((lstate.at(i).angle - meas_.at(j).angle));
            }

            it.at(i) = (std::min_element(dist.begin(), dist.end()));

            uint8_t it2 = it.at(i) - dist.begin();

            if(*it.at(i) < angles::from_degrees(30.0))
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



void CSoundGrid::clearStates()
{
    if(!meas_.empty()) meas_.clear();
    if(!cmeas_.empty()) cmeas_.clear();
    if(!cstate_.empty()) cstate_.clear();
    if(!match_meas_.empty()) match_meas_.clear();

}



void CSoundGrid::addMeasurements()
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

    ROS_ASSERT(cmeas_.size() == cstate_.size());
}


void CSoundGrid::filterStates()
{
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

            if(fabs(s1.angle - s2.angle) < angles::from_degrees(30.0))
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

void CSoundGrid::publishProbability()
{
    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.z = grid_->posterior.at(i);
    }

    if(prob_pub_.getNumSubscribers() > 0)
        prob_pub_.publish(prob_);
}

void CSoundGrid::publishOccupancyGrid()
{
    if(grid_pub_.getNumSubscribers() > 0)
        grid_pub_.publish(grid_->occupancy_grid);
}


void CSoundGrid::spin()
{
    if(!grid_->polar_array.predicted.empty()) grid_->polar_array.predicted.clear();

    ROS_INFO_COND(DEBUG,"--- spin ---");

    makeStates();
    updateKF();
    grid_->updateGrid(18);
    publishProbability();
    publishOccupancyGrid();

}

void CSoundGrid::updateKF()
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

        if(fabs(x.angle + M_PI) < 1e-2) x.angle = fabs(x.angle);

        if(x.var_angle < (float) angles::from_degrees(5.0) &&
                x.var_range < 35.0)
        {
            ROS_INFO_COND(DEBUG, "Checking the variance: angle: %f range: %f",
                          x.var_angle, x.var_range);
            grid_->polar_array.predicted.push_back(x);
        }
    }
}


CSoundGrid::~CSoundGrid()
{
    ROS_INFO("Deconstructing the constructed SoundGrid.");
    delete grid_;
    delete tf_listener_;
}
