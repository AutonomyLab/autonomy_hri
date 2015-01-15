#ifndef LIKELIHOOD_GRID_H
#define LIKELIHOOD_GRID_H
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
#include "grid.h"


class LikelihoodGrid
{
private:
    ros::NodeHandle n_;
    tf::TransformListener* tf_listener_;

    // Leg
    ros::Publisher predicted_leg_base_pub_;
    ros::Publisher last_leg_base_pub_;
    ros::Publisher current_leg_base_pub_;
    ros::Publisher legs_grid_pub_;
    Grid* leg_grid_;
    ros::Time last_leg_time_;
    ros::Duration leg_diff_time_;
    bool LEG_DETECTION_ENABLE_;
    nav_msgs::OccupancyGrid leg_occupancy_grid_;
    std::vector<PolarPose> current_leg_pose_array_base_polar_;
    std::vector<PolarPose> last_leg_pose_array_base_polar_;
    std::vector<PolarPose> predicted_leg_pose_array_base_polar_;


    // person
    ros::Publisher torso_grid_pub_;
    std::string torso_frame_id_;
    Grid* torso_grid_;
    ros::Time last_torso_time_;
    ros::Duration torso_diff_time_;
    bool TORSO_DETECTION_ENABLE_;
    nav_msgs::OccupancyGrid torso_occupancy_grid_;

    // Sound
    ros::Publisher sound_grid_pub_;
    std::string sound_frame_id_;
    Grid* sound_grid_;
    ros::Time last_sound_time_;
    ros::Duration sound_diff_time_;
    bool SOUND_DETECTION_ENABLE_;
    nav_msgs::OccupancyGrid sound_occupancy_grid_;


    // Periodic Gesture
    ros::Publisher periodic_grid_pub_;
    ros::Publisher max_periodic_prob_pub_;
    std::string periodic_frame_id_;
    Grid* periodic_grid_;
    ros::Time last_periodic_time_;
    ros::Duration periodic_diff_time_;
    bool PERIODIC_GESTURE_DETECTION_ENABLE_;
    nav_msgs::OccupancyGrid periodic_occupancy_grid_;

    // Human
    ros::Publisher human_grid_pub_;
    ros::Publisher local_maxima_pub_;
    ros::Publisher max_prob_pub_;
    std::string human_frame_id_;
    Grid* human_grid_;
    nav_msgs::OccupancyGrid human_occupancy_grid_;
    bool FUSE_MULTIPLY_;
    geometry_msgs::PoseArray local_maxima_;
    geometry_msgs::PointStamped maximum_probability_;
    geometry_msgs::PointStamped maximum_periodic_probability_;


    //encoder
    ros::Time encoder_last_time_;
    ros::Time last_time_;
    ros::Duration encoder_diff_time_;
    geometry_msgs::Pose diff_pose_crtsn_;
    PolarPose polar_diff_pose_;
    CellProbability_t CELL_PROBABILITY_;
    CellProbability_t LEG_CELL_PROBABILITY_;
    CellProbability_t TORSO_CELL_PROBABILITY_;
    CellProbability_t SOUND_CELL_PROBABILITY_;

    SensorFOV_t FOV_;
    Velocity_t robot_velocity_;
    bool MOTION_MODEL_ENABLE_;

    double prior_threshold_;
    double UPDATE_RATE_;
    double TARGET_DETETION_PROBABILITY_;
    double FALSE_POSITIVE_PROBABILITY_;
//    double robot_angular_velocity;
//    double robot_linear_velocity;
    double MAP_RESOLUTION_;

    int LOOP_RATE_;
    int number_of_sensors_;
    int SENSITIVITY_;
    int MAP_SIZE_;

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
    void occupancyGrid(Grid* grid,
                       nav_msgs::OccupancyGrid *occupancy_grid);

    void initLegGrid(SensorFOV_t _fov);              //NEEDED FOR EVERY HUMAN FEATURE
    void initTorsoGrid(SensorFOV_t _fov);
    void initSoundGrid(SensorFOV_t _fov);
    void initPeriodicGrid(SensorFOV_t _fov);
    void initHumanGrid(SensorFOV_t _fov);

public:
    ros::Time lk;
    LikelihoodGrid();
    LikelihoodGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener);

    void syncCallBack(const geometry_msgs::PoseArrayConstPtr& leg_msg,
                      const nav_msgs::OdometryConstPtr& encoder_msg);
    void soundCallBack(const hark_msgs::HarkSourceConstPtr &sound_msg);
    void torsoCallBack(const autonomy_human::raw_detectionsConstPtr &torso_msg);
    void periodicCallBack(const autonomy_human::raw_detectionsConstPtr &periodic_msg);

    void spin();
    ~LikelihoodGrid();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif
