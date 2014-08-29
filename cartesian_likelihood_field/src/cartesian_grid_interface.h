#ifndef CARTESIAN_GRID_INTERFACE_H
#define CARTESIAN_GRID_INTERFACE_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include "cartesian_grid.h"


class CartesianGridInterface
{
private:
    ros::NodeHandle n;
    tf::TransformListener* tf_listener;

    // Leg
    ros::Publisher predicted_leg_base_pub;
    ros::Publisher last_leg_base_pub;
    ros::Publisher current_leg_base_pub;
    ros::Publisher legs_grid_pub;
    CartesianGrid* leg_grid;
    ros::Time last_leg_time;
    ros::Duration leg_diff_time;
    bool LEG_DETECTION_ENABLE;
    nav_msgs::OccupancyGrid leg_occupancy_grid;
    std::vector<PolarPose> current_leg_pose_array_base_polar;
    std::vector<PolarPose> last_leg_pose_array_base_polar;
    std::vector<PolarPose> predicted_leg_pose_array_base_polar;


    // person
    ros::Publisher torso_grid_pub;
    std::string torso_frame_id;
    CartesianGrid* torso_grid;
    ros::Time last_torso_time;
    ros::Duration torso_diff_time;
    bool TORSO_DETECTION_ENABLE;
    nav_msgs::OccupancyGrid torso_occupancy_grid;

    // Sound
    ros::Publisher sound_grid_pub;
    std::string sound_frame_id;
    CartesianGrid* sound_grid;
    ros::Time last_sound_time;
    ros::Duration sound_diff_time;
    bool SOUND_DETECTION_ENABLE;
    nav_msgs::OccupancyGrid sound_occupancy_grid;


    // Periodic Gesture
    ros::Publisher periodic_grid_pub;
    ros::Publisher max_periodic_prob_pub;
    std::string periodic_frame_id;
    CartesianGrid* periodic_grid;
    ros::Time last_periodic_time;
    ros::Duration periodic_diff_time;
    bool PERIODIC_GESTURE_DETECTION_ENABLE;
    nav_msgs::OccupancyGrid periodic_occupancy_grid;

    // Human
    ros::Publisher human_grid_pub;
    ros::Publisher local_maxima_pub;
    ros::Publisher max_prob_pub;
    std::string human_frame_id;
    CartesianGrid* human_grid;
    nav_msgs::OccupancyGrid human_occupancy_grid;
    bool FUSE_MULTIPLY;
    geometry_msgs::PoseArray local_maxima;
    geometry_msgs::PointStamped maximum_probability;
    geometry_msgs::PointStamped maximum_periodic_probability;


    //encoder
    ros::Time encoder_last_time;
    ros::Time last_time;
    ros::Duration encoder_diff_time;
    geometry_msgs::Pose diff_pose_crtsn;
    PolarPose polar_diff_pose;
    CellProbability_t CELL_PROBABILITY;
    CellProbability_t LEG_CELL_PROBABILITY;
    CellProbability_t TORSO_CELL_PROBABILITY;
    CellProbability_t SOUND_CELL_PROBABILITY;

    SensorFOV_t FOV;
    Velocity_t robot_velocity;
    bool MOTION_MODEL_ENABLE;

    double prior_threshold;
    double UPDATE_RATE;
    double TARGET_DETETION_PROBABILITY;
    double FALSE_POSITIVE_PROBABILITY;
//    double robot_angular_velocity;
//    double robot_linear_velocity;
    double MAP_RESOLUTION;

    int LOOP_RATE;
    int number_of_sensors;
    int SENSITIVITY;
    int MAP_SIZE;

    void init();
    bool transformToBase(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped& target_point,
                         bool debug = false);
    bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                         geometry_msgs::PoseArray& target,
                         bool debug = false);
    bool transformToOdom(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped target_point,
                         bool debug = false);
    void occupancyGrid(CartesianGrid* grid,
                       nav_msgs::OccupancyGrid *occupancy_grid);

    void initLegGrid(SensorFOV_t _fov);              //NEEDED FOR EVERY HUMAN FEATURE
    void initTorsoGrid(SensorFOV_t _fov);
    void initSoundGrid(SensorFOV_t _fov);
    void initPeriodicGrid(SensorFOV_t _fov);
    void initHumanGrid(SensorFOV_t _fov);

public:
    ros::Time lk;
    CartesianGridInterface();
    CartesianGridInterface(ros::NodeHandle _n, tf::TransformListener* _tf_listener);

    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);
    void soundCallBack(const hark_msgs::HarkSourceConstPtr &sound_msg);
    void torsoCallBack(const autonomy_human::raw_detectionsConstPtr &torso_msg);
    void periodicCallBack(const autonomy_human::raw_detectionsConstPtr &periodic_msg);

    void spin();
    ~CartesianGridInterface();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif
