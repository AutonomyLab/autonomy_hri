#include "chumangrid.h"
#define DEBUG false
CHumanGrid::CHumanGrid()
{
}

CHumanGrid::CHumanGrid(ros::NodeHandle n):
    n_(n),
    initialized_(false),
    state_time_threshold_(10.0)
{
    init();
}

void CHumanGrid::init()
{
    human_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("human_grid", 10);
    highest_point_pub_ = n_.advertise<geometry_msgs::PointStamped>("maximum_probability", 10) ;
    local_maxima_pub_ = n_.advertise<geometry_msgs::PoseArray>("local_maxima",10);

    initGrid();
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
    last_hp_.point.z = hp_.point.z = 0.0;
    last_hp_.point.x = last_hp_.point.y = -10.0;
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
        grid_ = new CGrid(40, sfov, 0.5, cp, 0.9, 0.1);
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
    leg_weight = msg->data.at(0);
    sound_weight = msg->data.at(1);
    torso_weight = msg->data.at(2);

}

void CHumanGrid::predictLastHighestPoint()
{
    if(fabs(velocity_.linear) < 1e-2 && fabs(velocity_.angular) < 1e-2) return;

    ros::Duration dt = ros::Time::now() - last_time_;

    ROS_INFO("dt: %0.4f", dt.toSec());
    float dr = velocity_.linear * dt.toSec();
    float da = velocity_.angular * dt.toSec();

    float r1 = sqrt(last_hp_.point.x * last_hp_.point.x + last_hp_.point.y * last_hp_.point.y);
    float a1 = atan2(last_hp_.point.y, last_hp_.point.x);

    float r2 = r1 + dr;
    float a2 = a1 + da;

    last_hp_.point.x = r2 * cos(a2);
    last_hp_.point.y = r2 * sin(a2);

    ROS_INFO("Updating last state...");

}

void CHumanGrid::printFusedFeatures()
{
    std::string state = "NO FEATURE";
    float eps = 1e-3;
    if(hp_.point.z < (0.1667 + eps)){state = "LEG";}
    else if(hp_.point.z < (0.3334 + eps)) {state = "SOUND";}
    else if(hp_.point.z < (0.5 + eps)) {state = "TORSO OR LEG+SOUND";}
    else if(hp_.point.z < (0.6667 + eps)) {state = "LEG+TORSO";}
    else if(hp_.point.z < (0.8334 + eps)) {state = "SOUND+TORSO";}
    else {state = "ALL :)";}

    ROS_WARN("state is %s   %0.4f", state.c_str(), hp_.point.z);

}


void CHumanGrid::average()
{
    ros::Time now = ros::Time::now();
    float maxw = std::max(std::max(leg_max_, sound_max_), torso_max_);

    lw_  = (leg_max_ > 0.0) ? (maxw / leg_max_) : 1.0;
    sw_ = (sound_max_ > 0.0) ? (maxw / sound_max_) : 1.0;
    tw_ = (torso_max_ > 0.0) ? (maxw / torso_max_) : 1.0;

    float num = 0.0;
    float denum = (lw_ * leg_weight) + (sw_ * sound_weight) + (tw_ * torso_weight); //

    std::vector<float> temp;
    temp.resize(prob_.poses.size());

    float max = -1000;


    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        num =   lw_ * leg_weight * leg_prob_.poses.at(i).position.z +
                sw_ * sound_weight * sound_prob_.poses.at(i).position.z +
                tw_ * torso_weight * torso_prob_.poses.at(i).position.z;

        prob_.poses.at(i).position.z = num/denum;
        grid_->posterior.at(i) = num/denum;
        temp.at(i) = num/denum;
        max = std::max(max, temp.at(i));
    }

    for(size_t i = 0; i < grid_->grid_size; i++)
    {
        occupancy_grid_.data.at(i) = (int) 100 * (temp.at(i)) / max;
    }

    occupancy_grid_.header.stamp = now;
    human_grid_pub_.publish(occupancy_grid_);

    std::vector<float>::iterator it = std::max_element(temp.begin(), temp.end());

    ROS_INFO_COND(DEBUG,"max probability: %.4f ", max);



    // Using Local Maxima
//    grid_->updateLocalMaximas();
//    hp_.point = grid_->highest_prob_point.point;

    //Use highest Point
    hp_.point = prob_.poses.at(it - temp.begin()).position;

    if(!initialized_ && hp_.point.z > 0.0){ last_hp_ = hp_; initialized_ = true;}

    transitState();
    last_time_ = now;

    hp_.header.stamp = now;
    highest_point_pub_.publish(hp_);

    printFusedFeatures();
    publishLocalMaxima();
}

void CHumanGrid::resetState()
{
    state_time_ = ros::Time::now();
    last_hp_.point = hp_.point;
}

void CHumanGrid::transitState()
{
    ros::Duration dt = ros::Time::now() - state_time_;

    if(hp_.point.z <  0.4 ||
            fabs(hp_.point.z - last_hp_.point.z) < 0.01 * hp_.point.z)
    {
        if(dt.toSec() > state_time_threshold_)    {    ROS_INFO("11");
 resetState(); return;    }

        predictLastHighestPoint();
        hp_.point = last_hp_.point;
        ROS_INFO("12.");

    }
    else{ resetState();     ROS_INFO("2");}


    last_hp_.point = hp_.point;


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
