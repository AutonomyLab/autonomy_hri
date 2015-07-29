#include "chumangrid.h"
#define DEBUG false
#define STATE_DEBUG false

CHumanGrid::CHumanGrid()
{
}

CHumanGrid::CHumanGrid(ros::NodeHandle n, int _probability_projection_step):
    n_(n),
    initialized_(false),
    state_time_threshold_(10.0),
    probability_projection_step(_probability_projection_step)
{
    init();
}

CHumanGrid::CHumanGrid(ros::NodeHandle n, float lw, float sw, float tw, int _probability_projection_step):
    n_(n),
    initialized_(false),
    state_time_threshold_(5.0),
    leg_weight_(lw),
    sound_weight_(sw),
    torso_weight_(tw),
    probability_projection_step(_probability_projection_step)
{
    init();
}

void CHumanGrid::init()
{
    human_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("human/grid", 10);
    highest_point_pub_ = n_.advertise<geometry_msgs::PointStamped>("human/maximum_probability", 10) ;
    local_maxima_pub_ = n_.advertise<geometry_msgs::PoseArray>("human/local_maxima",10);
    proj_pub_ = n_.advertise<geometry_msgs::PoseArray>("human/projection",10);


    initGrid();
    calculateProbabilityThreshold();
    leg_prob_.poses.resize(1600); //TODO: FIX THIS
    sound_prob_.poses.resize(1600);
    torso_prob_.poses.resize(1600);
    occupancy_grid_.data.resize(grid_->grid_size, 0.0);
    occupancy_grid_.info.height = occupancy_grid_.info.width = 40;
    occupancy_grid_.info.resolution = 0.5;
    occupancy_grid_.info.origin.position.x = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.info.origin.position.y = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.header.frame_id = "base_footprint";
    hp_.header.frame_id = "base_footprint";
    tracked_hp_.point.z = hp_.point.z = 0.0;
    tracked_hp_.point.x = tracked_hp_.point.y = -10.0;
    hp_.point.x = hp_.point.y = -10.0;
    last_time_ = ros::Time::now();
    state_time_ = ros::Time::now();
}

void CHumanGrid::initGrid()
{
    CellProbability_t cp;
    cp.free = 0.1;
    cp.human = 0.9;
    cp.unknown = 0.5;

    SensorFOV_t sfov;
    sfov.range.max = 10.0;
    sfov.range.min = 0.0;
    sfov.angle.max = angles::from_degrees(180.0);
    sfov.angle.min = angles::from_degrees(-180.0);

    try
    {
        grid_ = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1, probability_projection_step);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new human Grid: bad_alloc caught: " << ba.what() << '\n';
    }
    prob_.poses.resize(grid_->grid_size);

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        prob_.poses.at(i).position.x = grid_->map.cell.at(i).cartesian.x;
        prob_.poses.at(i).position.y = grid_->map.cell.at(i).cartesian.y;
        prob_.poses.at(i).position.z = 0.0;
    }
    grid_->local_maxima_poses.header.frame_id = "base_footprint";
}

void CHumanGrid::legCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    leg_prob_.poses.clear();
    leg_prob_.poses = msg->poses;

    leg_max_ = 0.0;
    for(size_t i = 0; i < leg_prob_.poses.size(); i++)
    {
        leg_max_ = (leg_max_ > leg_prob_.poses.at(i).position.z)
                ? leg_max_ : leg_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::soundCallBack(const geometry_msgs::PoseArrayConstPtr& msg)
{
    sound_prob_.poses.clear();
    sound_prob_.poses = msg->poses;

    sound_max_ = 0.0;
    for(size_t i = 0; i < sound_prob_.poses.size(); i++)
    {
        sound_max_ = (sound_max_ > sound_prob_.poses.at(i).position.z)
                ? sound_max_ : sound_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::torsoCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    torso_prob_.poses.clear();
    torso_prob_.poses = msg->poses;

    torso_max_ = 0.0;
    for(size_t i = 0; i < torso_prob_.poses.size(); i++)
    {
        torso_max_ = (torso_max_ > torso_prob_.poses.at(i).position.z)
                ? torso_max_ : torso_prob_.poses.at(i).position.z;
    }
}

void CHumanGrid::encoderCallBack(const nav_msgs::OdometryConstPtr& msg)
{
    velocity_.angular = - msg->twist.twist.angular.z;
    velocity_.linear = - sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
            msg->twist.twist.linear.y * msg->twist.twist.linear.y);
    velocity_.lin.x = - msg->twist.twist.linear.x;
    velocity_.lin.y = - msg->twist.twist.linear.y;

    ROS_INFO_COND(DEBUG,"linear: %0.4f     angular: %0.4f",velocity_.linear, velocity_.angular);
}


void CHumanGrid::weightsCallBack(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    leg_weight_ = msg->data.at(0);
    sound_weight_ = msg->data.at(1);
    torso_weight_ = msg->data.at(2);
    calculateProbabilityThreshold();

}

void CHumanGrid::calculateProbabilityThreshold()
{
//    float u = std::min(leg_weight_ + sound_weight_,
//                               std::min(leg_weight_ + torso_weight_, torso_weight_ + sound_weight_));

    float eps = 0.01;

    probability_threshold_ = std::max(leg_weight_, std::max(sound_weight_, torso_weight_)) + eps;

    probability_threshold_ /= (sound_weight_ + leg_weight_ + torso_weight_);
}

void CHumanGrid::predictLastHighestPoint()
{
    if(fabs(velocity_.linear) < 1e-2 && fabs(velocity_.angular) < 1e-2) return;

    ros::Duration dt = ros::Time::now() - last_time_;

    float dr = velocity_.linear * dt.toSec();
    float da = velocity_.angular * dt.toSec();

    float r1 = sqrt(tracked_hp_.point.x * tracked_hp_.point.x + tracked_hp_.point.y * tracked_hp_.point.y);
    float a1 = atan2(tracked_hp_.point.y, tracked_hp_.point.x);

    float r2 = r1 + dr;
    float a2 = a1 + da;

    tracked_hp_.point.x = r2 * cos(a2);
    tracked_hp_.point.y = r2 * sin(a2);

    ROS_INFO("Updating last state...");

}

void CHumanGrid::printFusedFeatures()
{
    float sum = leg_weight_ + sound_weight_ + torso_weight_;

    std::string state = "NOT ENOUGH FEATURE";
    float eps = 1e-3;
    if(hp_.point.z < probability_threshold_ + eps ){state = "NOT ENOUGH FEATURE";}
//    else if(hp_.point.z < (torso_weight_/sum + eps)) {state = "TORSO";}
//    else if(hp_.point.z < (sound_weight_/sum + eps)) {state = "SOUND";}
    else if(hp_.point.z < ((leg_weight_ + torso_weight_)/sum + eps)) {state = "LEG + TORSO";}
    else if(hp_.point.z < ((leg_weight_ + sound_weight_)/sum + eps)) {state = "LEG + SOUND";}
    else if(hp_.point.z < ((torso_weight_ + sound_weight_)/sum + eps)) {state = "SOUND + TORSO";}
    else {state = "ALL :)";}

    ROS_WARN("* %s *  %0.4f", state.c_str(), hp_.point.z);

}

void CHumanGrid::publishProjection()
{
    if(proj_pub_.getNumSubscribers() > 0)
        proj_pub_.publish(grid_->grid_projection);
}


void CHumanGrid::integrateProbabilities()
{
    ros::Time now = ros::Time::now();

//    float maxw = std::max(std::max(leg_max_, sound_max_), torso_max_);
//    lw_  = (leg_max_ > 0.0) ? (maxw / leg_max_) : 1.0;
//    sw_ = (sound_max_ > 0.0) ? (maxw / sound_max_) : 1.0;
//    tw_ = (torso_max_ > 0.0) ? (maxw / torso_max_) : 1.0;

    lw_ = sw_ = tw_ = 1.0; //TODO: REMOVE THESE PARAMS

    float num = 0.0;
    float denum = (lw_ * leg_weight_) + (sw_ * sound_weight_) + (tw_ * torso_weight_); //

    std::vector<float> temp;
    temp.resize(prob_.poses.size());

    float max = -1000;


    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        num =   lw_ * leg_weight_ * leg_prob_.poses.at(i).position.z +
                sw_ * sound_weight_ * sound_prob_.poses.at(i).position.z +
                tw_ * torso_weight_ * torso_prob_.poses.at(i).position.z;

        prob_.poses.at(i).position.z = num/denum;
        temp.at(i) = grid_->posterior.at(i) = num/denum;
        max = std::max(max, temp.at(i));
    }

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        occupancy_grid_.data.at(i) = (int) 100 * (temp.at(i)) / max;
    }

    occupancy_grid_.header.stamp = now;
    human_grid_pub_.publish(occupancy_grid_);


    ROS_INFO_COND(DEBUG,"max probability: %.4f ", max);


    // Using Local Maxima
//    grid_->updateLocalMaximas();
//    hp_.point = grid_->highest_prob_point.point;

    //Use highest Point
    std::vector<float>::iterator it = std::max_element(temp.begin(), temp.end());
    hp_.point = prob_.poses.at(it - temp.begin()).position;

    transitState();
    last_time_ = now;

    hp_.header.stamp = now;
    highest_point_pub_.publish(hp_);

    printFusedFeatures();
    publishLocalMaxima();
    grid_->projectGrid();
    publishProjection();
}

void CHumanGrid::newState()
{
    state_time_ = ros::Time::now();
    tracked_hp_.point = hp_.point;
}

void CHumanGrid::resetState()
{
    state_time_ = ros::Time::now();
    hp_.point.z = 0.0;
    hp_.point.x = -10.0;
    hp_.point.y = -10.0;
    tracked_hp_.point = hp_.point;
}

void CHumanGrid::transitState()
{
    ros::Duration dt = ros::Time::now() - state_time_;

    ROS_INFO_COND(STATE_DEBUG,"time duration: %0.4f",dt.toSec());

    float eps = 0.01; // TODO: Make eps a param

    if(hp_.point.z - probability_threshold_ > eps)
    {
        ROS_INFO_COND(STATE_DEBUG,"two cues: 1");


        if(hp_.point.z - tracked_hp_.point.z > -0.05)
        {
            ROS_INFO_COND(STATE_DEBUG,"start new state: 3");
            newState();
        }

        else
        {
            ROS_INFO_COND(STATE_DEBUG,"should I track last state?: 4");

            if(dt.toSec() < state_time_threshold_)
            {
                ROS_INFO_COND(STATE_DEBUG,"Yes! track last state: 5");
                predictLastHighestPoint();
                hp_.point = tracked_hp_.point;
            }
            else
            {
                ROS_INFO_COND(STATE_DEBUG,"NO! reset everything: 6");
                resetState();
            }
        }
    }

    else
    {
        ROS_INFO_COND(STATE_DEBUG,"one cue: 2");

        if(tracked_hp_.point.z - probability_threshold_ > -1e-2)
        {
            ROS_INFO_COND(STATE_DEBUG,"should i still track last state? : 6");
            if(dt.toSec() < state_time_threshold_)
            {
                ROS_INFO_COND(STATE_DEBUG,"Yes! track last state: 5");
                predictLastHighestPoint();
                hp_.point = tracked_hp_.point;
            }
            else
            {
                ROS_INFO_COND(STATE_DEBUG,"NO! reset everything: 6");
                resetState();
            }
        }
        else
        {
            ROS_INFO_COND(STATE_DEBUG,"reset state :7");
            resetState();

        }
    }
}

void CHumanGrid::publishLocalMaxima()
{
    grid_->local_maxima_poses.header.stamp = ros::Time::now();
    local_maxima_pub_.publish(grid_->local_maxima_poses);
}

CHumanGrid::~CHumanGrid()
{
    ROS_INFO("Deconstructing the constructed Human Grid.");
    delete grid_;
}
