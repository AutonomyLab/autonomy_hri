#ifndef LEG_GRID_H
#define LEG_GRID_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "grid.h"
#include "kalmanfilter.h"

class LegGrid
{

private:

    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;
    ros::Publisher leg_grid_pub_;
    ros::Publisher predicted_leg_pub_;
    Velocity_t leg_velocity_;
    ros::Duration diff_time_;
    ros::Time last_time_;
    Grid* grid;
    KalmanFilter* kalman_filter;
    bool initialized;

    std::vector<PolarPose> current_state;
    std::vector<PolarPose> current_measurement;
    std::vector<bool> available_measurement;
    std::vector<uint8_t> no_measurement;

    void init();
    void makeStates();

    bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                         geometry_msgs::PoseArray& target,
                         bool debug = false);

public:

    LegGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener);
    ~LegGrid();

    /* TODO: use async callback */

    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);

    void spin();


};

#endif // LEG_GRID_H
