#ifndef CARTESIAN_GRID_H
#define CARTESIAN_GRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "polarcord.h"


struct PointXYP_t{
    double x;
    double y;
    double p;
    double dist(PointXYP_t target){return sqrt(pow(x-target.x,2) + pow(y - target.y,2));}
};

struct FOV_t{
    double min;
    double max;
    double res;
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
float_t resolution;

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
    //void initGrid(MapMetaData_t map);

public:
    MapMetaData_t map;
    uint32_t grid_size;
    float* posterior;
    float* prior;
    float* likelihood;
    SensorFOV_t sensor_fov;
    bool flag;
    CellProbability_t cell_prob;
    CartesianGrid(uint32_t map_size,
                  float_t map_resolution,
                  SensorFOV_t _sensor_fov,
                  CellProbability_t cell_prob);
    CartesianGrid();
    ~CartesianGrid();
    void fuse(float* input);
    void computeLikelihood(const std::vector<PolarPose>& pose, float* data, const double std_range,const double std_angle);
    void updateGridProbability(float* pr, float* lk, float* po);
    void copyProbability(float* input, float* output);
    void toLogOdd(float* probability_data, float* logodd_data);
    void fromLogOdd(float* logodd_data, float* probability_data);
    void scaleProbability(float* data, float s);
    void setGridProbability(float* data, float val);
    void setUnknownProbability(float* data, float val);
    void setFreeProbability(float* data, float val);
    float minProbability(float* data);
    float maxProbability(float* data);

};

#endif
