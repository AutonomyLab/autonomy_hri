#ifndef CARTESIAN_GRID_H
#define CARTESIAN_GRID_H

#include <vector>
#include <cmath>
//#include <math.h>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <hark_msgs/HarkSource.h>
#include <hark_msgs/HarkSourceVal.h>
#include <autonomy_human/human.h>
#include <autonomy_human/raw_detections.h>
#include "polarcord.h"


double normalize(const double val, const double x_min, const double x_max, const double range_min, const double range_max);

struct FOV_t{
    double min;
    double max;
    //double res;
};

struct SensorFOV_t
{
    FOV_t range;
    FOV_t angle;
};

struct CellProbability_t{
    double free;
    double unknown;
    double human;
};

struct LocalMaxima_t{
    size_t index;
    bool tracking;
    int8_t counter;
};

struct MapMetaData_t{
// This hold basic information about the characterists of the CartesianGrid

// The time at which the map was loaded
ros::Time map_load_time;

// The map resolution [m/cell]
double resolution;

//Map width [cells]
uint32_t width;

// Map height [cells]
uint32_t height;

// The origin of the map [m, m, rad].  This is the real-world pose of the
// cell (0,0) in the map.
geometry_msgs::Pose origin;

// robot-centric position of the cell (r,c) in the map
std::vector<geometry_msgs::Point> cell_crtsn;
std::vector<PolarPose> cell_polar;

// is cell(r,c) in sensor fov?
std::vector<bool> cell_inFOV;

//// cell number
//size_t cell_num;
};

template <typename T>
struct Time_t{
    T current;
    T past;
    T predicted;
};


class CartesianGrid
{
private:
    FOV_t x;
    FOV_t y;

    size_t maxProbCellNum();
    double cellsDistance(size_t c1, size_t c2);
    std::vector<LocalMaxima_t> old_lm;
    std::vector<LocalMaxima_t> new_lm;
    std::vector<LocalMaxima_t> matched_lm;
    SensorFOV_t sensor_fov;
    std::vector<double> true_likelihood;
    std::vector<double> false_likelihood;
    std::vector<double> predicted_posterior;
    std::vector<double> pred_true_likelihood;
    std::vector<double> pred_false_likelihood;
    LocalMaxima_t last_highest_lm;
    ros::Time past_time;
    ros::Duration diff_time;

    double angular_velocity;
    double linear_velocity;
    double target_detection_prob;
    double false_positive_prob;
    double max_probability;

    void predictLikelihood(const std::vector<PolarPose>& pose,
                                std::vector<double> &lk_true,
                                std::vector<double> &lk_false);
    void computeLikelihood(const std::vector<PolarPose>& pose,
                                std::vector<double> &lk_true,
                                std::vector<double> &lk_false);
    void updateGridProbability(std::vector<double> &pr,
                               std::vector<double> &true_lk,
                               std::vector<double> &false_lk,
                               std::vector<double> &po);
    void setOutFOVProbability(std::vector<double>& data, const double val);
    void setInFOVProbability(std::vector<double>& data, const double val);
    void updateVelocity(double robot_linear_velocity, double robot_angular_velocity,
                        double last_polar_range, double last_polar_angle);
    void getLocalMaximas();
    void trackLocalMaximas();

public:
    MapMetaData_t map;
    uint32_t grid_size;
    PolarPose stdev;
    CellProbability_t cell_prob;

    std::vector<LocalMaxima_t> main_lm;
    std::vector<double> posterior;
    std::vector<double> prior;

    Time_t<std::vector<PolarPose> > polar_array;

    Time_t<geometry_msgs::PoseArray> crtsn_array;
    geometry_msgs::PoseArray local_maxima_poses;
    geometry_msgs::PointStamped highest_prob_point;


    CartesianGrid(uint32_t map_size,
                  double_t map_resolution,
                  SensorFOV_t _sensor_fov,
                  CellProbability_t _cell_prob,
                  double target_detection_probability,
                  double false_positive_probability);
    CartesianGrid();
    ~CartesianGrid();

    void fuse(const std::vector<double> data_1, const std::vector<double> data_2,
              const std::vector<double> data_3, bool multiply);


    void bayesOccupancyFilter();
    void getPose(geometry_msgs::PoseArray &crtsn_array);
    void getPose(const autonomy_human::raw_detectionsConstPtr torso_img);
    void getPose(const hark_msgs::HarkSourceConstPtr& sound_src);

    void predict(double robot_linear_velocity, double robot_angular_velocity);
    void polar2Crtsn(std::vector<PolarPose> &polar_array,
                     geometry_msgs::PoseArray &crtsn_array);

    void updateLocalMaximas();
    void trackMaxProbability();
};

#endif