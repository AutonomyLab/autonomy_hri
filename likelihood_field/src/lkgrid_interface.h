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
    ros::Publisher legsLKGrid_pub;
    std::string leg_frame;
    LikelihoodGrid* legGrid;
    ros::Time lastLegsTime;
    ros::Duration diffLegs;
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
                            float _free_cell_probability);
    void update_legs(GridFOV_t sensorGridFOV);
    void legs_cb(const geometry_msgs::PoseArray& msg);
    //void legs_lkgrid_pub
    void spin(float cycle_time);
    void publish();
    ~LikelihoodGridInterface();
};


#endif
