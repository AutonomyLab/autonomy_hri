#ifndef LEG_GRID_H
#define LEG_GRID_H
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "grid.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <std_msgs/Float32MultiArray.h>

class CLegGrid
{

private:

  ros::NodeHandle n_;
  tf::TransformListener* tf_listener_;
  ros::Publisher grid_pub_;
  ros::Publisher predicted_leg_pub_;
  ros::Publisher prob_pub_;
  ros::Publisher proj_pub_;
  Velocity_t velocity_;
  ros::Duration diff_time_;
  ros::Time last_time_;
  ros::Time last_seen_leg_;
  CGrid* grid_;
  cv::KalmanFilter KFTracker_;
  cv::Mat KFmeasurement_;
  std::vector<PolarPose> cstate_;
  std::vector<PolarPose> cmeas_;
  std::vector<PolarPose> meas_;
  std::vector<bool> match_meas_;
  nav_msgs::Odometry encoder_reading_;
  geometry_msgs::PoseArray prob_;
//    geometry_msgs::PoseArray legs_reading_;
  geometry_msgs::PoseArray filtered_legs_;
  geometry_msgs::PoseArray base_footprint_legs_;
  int probability_projection_step;

  void init();
  void initKF();
  void initTfListener();
  void initGrid();
  void callbackClear();
  void computeObjectVelocity();
  void makeStates();
  void clearStates();
  void addLastStates();
  void addMeasurements();
  void filterStates();
  void updateKF();
  void publishPredictedLegs();
  void publishProbability();
  void publishOccupancyGrid();
  void filterLegs();
  void keepLastLegs();
  void publishProjection();

  bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                       geometry_msgs::PoseArray& target,
                       bool debug = false);

public:

  CLegGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener, int _probability_projection_step);
  ~CLegGrid();

  /* TODO: use async callback */

//    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
//                      const nav_msgs::OdometryConstPtr& encoder_msg);

  void legsCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg);

  void encoderCallBack(const nav_msgs::OdometryConstPtr& encoder_msg);

  void spin();


};

#endif // LEG_GRID_H
