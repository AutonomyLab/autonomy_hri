#ifndef CHUMANGRID_H
#define CHUMANGRID_H

#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PointStamped.h>
#include<nav_msgs/Odometry.h>
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
    ros::Publisher proj_pub_;

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

    float leg_weight_;
    float sound_weight_;
    float torso_weight_;

    float state_time_threshold_;
    float probability_threshold_;

    ros::Time last_time_;
    ros::Time state_time_;

    bool initialized_;

    Velocity_t velocity_;

    nav_msgs::OccupancyGrid occupancy_grid_;
    geometry_msgs::PointStamped hp_;
    geometry_msgs::PointStamped tracked_hp_;
    int probability_projection_step;

    CGrid* grid_;

    void init();
    void initGrid();
    void transitState();
    void printFusedFeatures();
    void predictLastHighestPoint();

    void publishLocalMaxima();
    void newState();
    void resetState();
    void calculateProbabilityThreshold();
    void publishProjection();

public:

    CHumanGrid();
    CHumanGrid(ros::NodeHandle n, int probability_projection_step);
    CHumanGrid(ros::NodeHandle n, float lw, float sw, float tw, int probability_projection_step);
    void integrateProbabilities();

    void legCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    void soundCallBack(const geometry_msgs::PoseArrayConstPtr &msg);
    void torsoCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    void weightsCallBack(const std_msgs::Float32MultiArrayConstPtr& msg);
    void encoderCallBack(const nav_msgs::OdometryConstPtr& msg);

    ~CHumanGrid();
};

#endif // CHUMANGRID_H
