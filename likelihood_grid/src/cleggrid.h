#ifndef LEG_GRID_H
#define LEG_GRID_H
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "grid.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

class CLegGrid
{

private:

    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;
    ros::Publisher leg_grid_pub_;
    ros::Publisher predicted_leg_pub_;
    Velocity_t leg_velocity_;
    ros::Duration diff_time_;
    ros::Time last_time_;
    CGrid* grid;
    cv::KalmanFilter KFTracker;
    cv::Mat KFmeasurement;
    std::vector<PolarPose> cstate;
    std::vector<PolarPose> cmeas;

    void init();
    void makeStates();

    bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                         geometry_msgs::PoseArray& target,
                         bool debug = false);

public:

    CLegGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener);
    ~CLegGrid();

    /* TODO: use async callback */

    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);

    void spin();


};

#endif // LEG_GRID_H
