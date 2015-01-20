#ifndef LEG_GRID_H
#define LEG_GRID_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "grid.h"

class LegGrid
{
private:
    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;
    ros::Publisher leg_grid_pub_;
    Velocity_t leg_velocity_;
    ros::Duration diff_time_;
    ros::Time last_time_;
    Grid* grid;
//    geometry_msgs::TwistWithCovariance velocity_;
    void init();
    void spin();
    void kalmanFilter();
public:
    LegGrid();
    bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                         geometry_msgs::PoseArray& target,
                         bool debug = false);
    /* TODO: use async callback */
    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);

};

#endif // LEG_GRID_H
