#ifndef CHUMANGRID_H
#define CHUMANGRID_H

#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PointStamped.h>
#include<std_msgs/Float32MultiArray.h>
#include"grid.h"

class CHumanGrid
{
private:
    ros::NodeHandle n_;
    ros::Publisher human_grid_pub_;
    ros::Publisher highest_point_pub_;

    geometry_msgs::PoseArray leg_prob_;
    geometry_msgs::PoseArray sound_prob_;
    geometry_msgs::PoseArray torso_prob_;
    geometry_msgs::PoseArray human_prob_;

    float lw_;
    float sw_;
    float tw_;

    nav_msgs::OccupancyGrid occupancy_grid_;
    geometry_msgs::PointStamped hp_;


    void init();

public:
    int leg_weight;
    int sound_weight;
    int torso_weight;
    CHumanGrid();
    CHumanGrid(ros::NodeHandle n);
    void average();

    void legCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    void soundCallBack(const geometry_msgs::PoseArrayConstPtr &msg);
    void torsoCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    ~CHumanGrid();
};

#endif // CHUMANGRID_H
