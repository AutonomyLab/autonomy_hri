#ifndef CHUMANGRID_H
#define CHUMANGRID_H

#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PointStamped.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/UInt8MultiArray.h>
#include"grid.h"

class CHumanGrid
{
private:
    ros::NodeHandle n_;
    ros::Publisher human_grid_pub_;
    ros::Publisher highest_point_pub_;
    ros::Publisher local_maxima_pub_;

    geometry_msgs::PoseArray leg_prob_;
    geometry_msgs::PoseArray sound_prob_;
    geometry_msgs::PoseArray torso_prob_;
    geometry_msgs::PoseArray prob_;

    float lw_;
    float sw_;
    float tw_;

    float leg_max_;
    float sound_max_;
    float torso_max_;

    nav_msgs::OccupancyGrid occupancy_grid_;
    geometry_msgs::PointStamped hp_;
    geometry_msgs::PointStamped last_hp_;


    CGrid* grid_;

    void init();
    void initGrid();

public:
    double leg_weight;
    double sound_weight;
    double torso_weight;
    CHumanGrid();
    CHumanGrid(ros::NodeHandle n);
    void average();

    void legCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    void soundCallBack(const geometry_msgs::PoseArrayConstPtr &msg);
    void torsoCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    void weightsCallBack(const std_msgs::Float32MultiArrayConstPtr& msg);
    ~CHumanGrid();
};

#endif // CHUMANGRID_H
