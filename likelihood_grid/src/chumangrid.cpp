#include "chumangrid.h"

CHumanGrid::CHumanGrid()
{
}

CHumanGrid::CHumanGrid(ros::NodeHandle n):
    n_(n)
{
    init();
}

void CHumanGrid::init()
{
    human_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("human_grid", 10);
    highest_point_pub_ = n_.advertise<geometry_msgs::PointStamped>("highest_probability_point", 10) ;
    leg_prob_.poses.resize(1600); //TODO: FIX THIS
    sound_prob_.poses.resize(1600);
    torso_prob_.poses.resize(1600);
    occupancy_grid_.info.height = occupancy_grid_.info.width = 40;
    occupancy_grid_.info.resolution = 0.5;
    occupancy_grid_.info.origin.position.x = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.info.origin.position.y = (float) 40 * 0.5 / -2.0;
    occupancy_grid_.header.frame_id = "base_footprint";
    hp_.header.frame_id = "base_footprint";
}

void CHumanGrid::legCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    if(!msg->poses.empty())
    {
        lw_ = leg_weight;
        leg_prob_.poses = msg->poses;
    }
    else
    {
        lw_ = 0.0;
        leg_prob_.poses.resize(1600);
    }
}

void CHumanGrid::soundCallBack(const geometry_msgs::PoseArrayConstPtr& msg)
{
    if(!msg->poses.empty())
    {
        sw_ = sound_weight;
        sound_prob_.poses = msg->poses;
    }
    else
    {
        sw_ = 0.0;
        sound_prob_.poses.resize(1600);
    }
}

void CHumanGrid::torsoCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{

    if(!msg->poses.empty())
    {
        tw_ = torso_weight;
        torso_prob_.poses = msg->poses;
    }
    else
    {
        torso_prob_.poses.resize(1600);
        tw_ = 0.0;
    }
}

void CHumanGrid::average()
{

    human_prob_.poses.resize(leg_prob_.poses.size());
    occupancy_grid_.data.resize(leg_prob_.poses.size(), 0.0);

    float num = 0.0;
    float denum = lw_ + tw_ + sw_; //

    std::vector<float> temp;
    temp.resize(human_prob_.poses.size());

    for(size_t i = 0; i < leg_prob_.poses.size(); i++)
    {
        num =   lw_ * leg_prob_.poses.at(i).position.z +
                sw_ * sound_prob_.poses.at(i).position.z +
                tw_ * torso_prob_.poses.at(i).position.z;

        human_prob_.poses.at(i).position.z = num/denum;
        temp.at(i) = num/denum;
        occupancy_grid_.data.at(i) = (int) 1000 * (num / denum);
    }
    occupancy_grid_.header.stamp = ros::Time::now();
    human_grid_pub_.publish(occupancy_grid_);
    std::vector<float>::iterator it = std::max_element(temp.begin(), temp.end());

    hp_.header.stamp = ros::Time::now();
    hp_.point = leg_prob_.poses.at(it - temp.begin()).position;
    hp_.point.z = human_prob_.poses.at(it - temp.begin()).position.z;

    highest_point_pub_.publish(hp_);
}

CHumanGrid::~CHumanGrid()
{
}
