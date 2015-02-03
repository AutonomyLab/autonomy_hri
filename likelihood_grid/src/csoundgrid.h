#ifndef CSOUNDGRID_H
#define CSOUNDGRID_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "grid.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <hark_msgs/HarkSource.h>

class CSoundGrid
{
public:

    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;
    ros::Publisher sound_grid_pub_;
//    ros::Publisher predicted_sound_pub_;
    Velocity_t sound_velocity_;
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

    CSoundGrid();
    CSoundGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener);
    ~CSoundGrid();

    void syncCallBack(const hark_msgs::HarkSourceConstPtr& sound_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);

    void spin();

};

#endif // CSOUNDGRID_H
