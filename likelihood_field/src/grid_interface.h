#ifndef GRID_INTERFACE_H
#define GRID_INTERFACE_H
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <hark_msgs/HarkSource.h>
#include <hark_msgs/HarkSourceVal.h>
#include <autonomy_human/human.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/ChannelFloat32.h>
#include "grid.h"


class GridInterface
{
private:
    ros::NodeHandle n;
    tf::TransformListener* tf_listener;
    // Leg
    ros::Publisher legs_grid_pub;
    std::string leg_frame_id;
    Grid* leg_grid;
    ros::Time last_leg_time;
    ros::Duration diff_leg_time;
    bool leg_detection_enable;
    unsigned int leg_counter;

    // Face
    ros::Publisher face_grid_pub;
    std::string face_frame_id;
    Grid* face_grid;
    ros::Time last_face_time;
    ros::Duration diff_face_time;
    bool torso_detection_enable;
    unsigned int torso_counter;


    // Sound
    ros::Publisher sound_grid_pub;
    std::string sound_frame_id;
    Grid* sound_grid;
    ros::Time last_sound_time;
    ros::Duration diff_sound_time;
    bool sound_detection_enable;
    unsigned int sound_counter;

    // Laser
    ros::Publisher laser_grid_pub;
    std::string laser_frame_id;
    Grid* laser_grid;
    ros::Time last_laser_time;
    ros::Duration diff_laser_time;
    bool laser_detection_enable;
    unsigned int laser_counter;

    // Human
    ros::Publisher human_grid_pub;
    std::string human_frame_id;
    Grid* human_grid;
    PointRAP_t* world_base;
    PointRAP_t* world_odom;

    double prior_threshold;
    double update_rate;
    double update_time_ratio;
    int number_of_sensors;
    int sensitivity;
    CellProbability_t cell_probability;
    GridFOV_t global_fov;

    void init();
    void initWorldGrids();
    bool transformToBase(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped& target_point,
                         bool debug = false);
    bool transformToOdom(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped target_point,
                         bool debug = false);
    sensor_msgs::PointCloud pointCloudGrid(Grid *polar_grid);
    void initLegs(GridFOV_t sensor_fov);              //NEEDED FOR EVERY HUMAN FEATURE
    void initFaces(GridFOV_t sensor_fov);
    void initSound(GridFOV_t sensor_fov);
    void initLaser(GridFOV_t sensor_fov);
    void initHuman();
    void publish();                                         // TO BE MODIFIED FOR EVERY HUMAN FEATURE

public:
    GridInterface();
    GridInterface(ros::NodeHandle _n, tf::TransformListener* _tf_listener);   // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    void legCallBack(const geometry_msgs::PoseArray& msg);      //NEEDED FOR EVERY HUMAN FEATURE
    void faceCallBack(const autonomy_human::human& msg);
    void soundCallBack(const hark_msgs::HarkSource& msg);
    void laserCallBack(const sensor_msgs::LaserScan& msg);
    void spin();                            // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    ~GridInterface();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif
