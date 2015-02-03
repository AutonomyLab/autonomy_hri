#ifndef CVISIONGRID_H
#define CVISIONGRID_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "grid.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

class CVisionGrid
{
    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;
    ros::Publisher vision_grid_pub_;
    Velocity_t torso_velocity_;
    ros::Duration diff_time_;
    ros::Time last_time_;
    CGrid* grid;
    cv::KalmanFilter KFTracker;
    cv::Mat KFmeasurement;
    std::vector<PolarPose> cstate;
    std::vector<PolarPose> cmeas;

    void init();
    void makeStates();


public:

    CVisionGrid();
    CVisionGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener);
    ~CVisionGrid();

    void syncCallBack(const autonomy_human::raw_detectionsConstPtr& torso_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);

    void spin();

};

#endif // CVISIONGRID_H
