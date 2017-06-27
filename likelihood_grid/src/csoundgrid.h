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
#include <std_msgs/Float32MultiArray.h>

class CSoundGrid
{
private:

  ros::NodeHandle n_;
  tf::TransformListener* tf_listener_;
  ros::Publisher grid_pub_;
  ros::Publisher prob_pub_;
  ros::Publisher proj_pub_;
  ros::Publisher marker_pub_;
  Velocity_t velocity_;
  ros::Duration diff_time_;
  ros::Time last_time_;
  ros::Time last_heard_sound_;
  CGrid* grid_;
  cv::KalmanFilter KFTracker_;
  cv::Mat KFmeasurement_;
  std::vector<PolarPose> cstate_;
  std::vector<PolarPose> cmeas_;
  nav_msgs::Odometry encoder_reading_;
  std::vector<hark_msgs::HarkSourceVal> ss_reading_;
  PolarPose polar_ss_;
  std::vector<PolarPose> meas_;
  std::vector<bool> match_meas_;
  geometry_msgs::PoseArray prob_;
  int probability_projection_step;
  visualization_msgs::MarkerArray marker_array;
  double power_threshold;

  void init();
  void initKF();
  void initGrid();
  void initTfListener();

  void callbackClear();
  void computeObjectVelocity();
  void rejectNotValidSoundSources(float p);
  void addMirrorSoundSource();
  void addSoundSource();
  void keepLastSound();

  void makeStates();
  void addLastStates();
  void clearStates();
  void addMeasurements();
  void filterStates();

  void updateKF();
  void publishProbability();
  void publishOccupancyGrid();
  void passStates();

  void publishProjection();
  void publishMarkers();


public:

  CSoundGrid();
  CSoundGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener, int _probability_projection_step, float _power_threshold);
  ~CSoundGrid();

  void syncCallBack(const hark_msgs::HarkSourceConstPtr& sound_msg,
                    const nav_msgs::OdometryConstPtr& encoder_msg);

  void spin();

};

#endif // CSOUNDGRID_H
