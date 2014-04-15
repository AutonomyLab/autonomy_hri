#ifndef LKGRID_INTERFACE_H
#define LKGRID_INTERFACE_H
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <autonomy_human/human.h>
#include "lkgrid.h"


class LikelihoodGridInterface
{
private:
    ros::NodeHandle n;
    // Leg
    ros::Publisher legsLKGrid_pub;
    std::string leg_frame;
    LikelihoodGrid* legGrid;
    ros::Time lastLegsTime;
    ros::Duration diffLegs;
    // Face
    ros::Publisher facesLKGrid_pub;
    std::string face_frame;
    LikelihoodGrid* faceGrid;
    ros::Time lastFacesTime;
    ros::Duration diffFaces;
    // Human
    ros::Publisher humanLKGrid_pub;
    std::string human_frame;
    LikelihoodGrid* humanGrid;
    //ros::Time lastFacesTime;
    //ros::Duration diffFaces;
    float update_rate;
    float update_time_ratio;
    float free_cell_probability;
    GridFOV_t globalGridFOV;
public:
    LikelihoodGridInterface();
    LikelihoodGridInterface(ros::NodeHandle _n,
                            GridFOV_t _globalGridFOV,
                            float _update_rate,
                            float _update_time_ratio,
                            float _free_cell_probability);   // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    void init_legs(GridFOV_t sensorGridFOV);              //NEEDED FOR EVERY HUMAN FEATURE
    void legs_cb(const geometry_msgs::PoseArray& msg);      //NEEDED FOR EVERY HUMAN FEATURE
    void init_faces(GridFOV_t sensorGridFOV);
    void faces_cb(const autonomy_human::human& msg);
    void init_human();
    void spin(float cycle_time);                            // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    void publish();                                         // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    sensor_msgs::PointCloud pc_grid(LikelihoodGrid *polar_grid);
    ~LikelihoodGridInterface();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif
