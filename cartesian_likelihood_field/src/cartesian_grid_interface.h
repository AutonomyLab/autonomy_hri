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
    ros::Publisher leg_occupancy_grid_pub;
    CartesianGrid* leg_grid;
    ros::Time last_leg_time;
    ros::Duration leg_diff_time;
    bool leg_detection_enable;
    unsigned int leg_counter;
    sensor_msgs::PointCloud leg_pointcloud_grid;
    nav_msgs::OccupancyGrid leg_occupancy_grid;
    std::vector<PolarPose> current_leg_pose_array_base_polar;
    std::vector<PolarPose> last_leg_pose_array_base_polar;
    std::vector<PolarPose> predicted_leg_pose_array_base_polar;


    // Face
    ros::Publisher torso_grid_pub;
    ros::Publisher torso_occupancy_grid_pub;
    std::string torso_frame_id;
    CartesianGrid* torso_grid;
    ros::Time last_torso_time;
    ros::Duration torso_diff_time;
    bool torso_detection_enable;
    unsigned int torso_counter;
    sensor_msgs::PointCloud torso_pointcloud_grid;
    nav_msgs::OccupancyGrid torso_occupancy_grid;



    // Sound
    ros::Publisher sound_grid_pub;
    ros::Publisher sound_occupancy_grid_pub;
    std::string sound_frame_id;
    CartesianGrid* sound_grid;
    ros::Time last_sound_time;
    ros::Duration sound_diff_time;
    bool sound_detection_enable;
    unsigned int sound_counter;
    sensor_msgs::PointCloud sound_pointcloud_grid;
    nav_msgs::OccupancyGrid sound_occupancy_grid;

    // Human
    ros::Publisher human_grid_pub;
    ros::Publisher human_occupancy_grid_pub;
    std::string human_frame_id;
    CartesianGrid* human_grid;
    //ros::Publisher highest_point_pub;
    unsigned int accept_counter;
    unsigned int reject_counter;
    sensor_msgs::PointCloud human_pointcloud_grid;
    nav_msgs::OccupancyGrid human_occupancy_grid;
    bool fuse_multiply;

    //encoder
    ros::Time last_time_encoder;
    ros::Time current_time_encoder;
    geometry_msgs::Pose diff_pose_crtsn;
    PolarPose polar_diff_pose;
    double diff_time;
    double robot_angular_velocity;
    double robot_linear_velocity;
    bool motion_model_enable;

    double prior_threshold;
    double update_rate;
    int loop_rate;
    int number_of_sensors;
    int sensitivity;
    CellProbability_t cell_probability;
    SensorFOV_t fov;

    int map_size;
    double map_resolution;

    void init();
    void initWorldGrids();
    bool transformToBase(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped& target_point,
                         bool debug = false);
    bool transformToBase(const geometry_msgs::PoseArrayConstPtr& source,
                         geometry_msgs::PoseArray& target,
                         bool debug = false);
    bool transformToOdom(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped target_point,
                         bool debug = false);
    void pointCloudGrid(CartesianGrid* grid,
                        sensor_msgs::PointCloud* pointcloud_grid);
    void occupancyGrid(CartesianGrid* grid,
                       nav_msgs::OccupancyGrid *occupancy_grid);

    void initLegs(SensorFOV_t sensor_fov);              //NEEDED FOR EVERY HUMAN FEATURE
    void initTorso(SensorFOV_t sensor_fov);
    void initSound(SensorFOV_t sensor_fov);
    void initHuman();

public:
    CartesianGridInterface();
    CartesianGridInterface(ros::NodeHandle _n, tf::TransformListener* _tf_listener);
    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg,
                      const autonomy_human::raw_detectionsConstPtr torso_msg,
                      const hark_msgs::HarkSourceConstPtr &sound_msg);

//    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
//                      const nav_msgs::OdometryConstPtr& encoder_msg);
    void spin();
    ~CartesianGridInterface();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif
