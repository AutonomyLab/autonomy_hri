#ifndef CARTESIAN_GRID_H
#define CARTESIAN_GRID_H

#include <vector>
//#include <array>
#include <cmath>
//#include <math.h>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "polarcord.h"


//struct PointXYP_t{
//    double x;
//    double y;
//    double p;
//    double dist(PointXYP_t target){return sqrt(pow(x-target.x,2) + pow(y - target.y,2));}
//};

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
std::vector<geometry_msgs::Point> cell_cart_pos;
std::vector<PolarPose> cell_pol_pos;

// is cell(r,c) in sensor fov?
std::vector<bool> cell_inFOV;
};


class CartesianGrid
{
private:
    FOV_t x;
    FOV_t y;
    void toLogOdd(std::vector<float> &pr_data, std::vector<float> &logodd_data);
    void fromLogOdd(std::vector<float> &logodd_data, std::vector<float> &pr_data);

public:
    MapMetaData_t map;
    uint32_t grid_size;
    std::vector<float> posterior;
    std::vector<float> prior;
    std::vector<float> likelihood;
    std::vector<float> predicted_posterior;
    std::vector<float> predicted_likelihood;

    bool flag;
    SensorFOV_t sensor_fov;
    CellProbability_t cell_prob;
    CartesianGrid(uint32_t map_size,
                  float_t map_resolution,
                  SensorFOV_t _sensor_fov,
                  CellProbability_t _cell_prob);
    CartesianGrid();
    ~CartesianGrid();
    void fuse(float* input);
    void computeLikelihood(const std::vector<PolarPose>& pose, std::vector<float> &data, const float std_range, const float std_angle);
    void updateGridProbability(std::vector<float>& pr, std::vector<float>& lk, std::vector<float>& po);
    void scaleProbability(float* data, float s);
    void setUnknownProbability(std::vector<float>& data, const float val);
    void setFreeProbability(std::vector<float>& data, const float val);
    void bayesOccupancyFilter(const std::vector<PolarPose> &pose, const float std_range, const float std_angle);
};

#endif
