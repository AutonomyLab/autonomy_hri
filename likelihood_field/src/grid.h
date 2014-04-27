#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include "polarcord.h"

double toRadian(double t);

struct PointRAP_t{
    double range;
    double angle;
    double probability;
    double distanceRange(double target){return fabs(range - target);}
    double distanceAngle(double target) {return fabs(angle - target);}
};

struct FOV_t{
    double min;
    double max;
    double resolution;
};

struct GridFOV_t
{
    FOV_t range;
    FOV_t angle;
    size_t getRowSize() {return round((range.max - range.min)/range.resolution) + 1;}
    size_t getColSize() {return round((angle.max - angle.min)/angle.resolution) + 1;}
    size_t getSize() {return getRowSize()*getColSize();}
};

struct CellProbability_t{
    double free;
    double unknown;
    double human;
};


class Grid
{
public:
    GridFOV_t global_fov;
    GridFOV_t sensor_fov;
    CellProbability_t cell_probability;
private:
    PointRAP_t* old_data;
    bool* is_known;
    void init();
    void initGrid(PointRAP_t* _data, const double val);
    void setUnknownArea();
    void freeProbability(PointRAP_t* _data);
    void unknownProbability(PointRAP_t* _data);
    void scaleProbability(PointRAP_t* _data, double s);
    double minProbability(PointRAP_t* _data);
    double maxProbability(PointRAP_t* _data);
    void copyProbability(PointRAP_t* source, PointRAP_t* target);
    void regionNumber(PointRAP_t* _data,
                            u_int8_t* region_data,
                            u_int8_t range_region_num,
                            u_int8_t angle_region_num);
public:
    PointRAP_t* data;
    PointRAP_t* new_data;
    bool flag;
    Grid();
    Grid(const GridFOV_t _sensor_fov,
                   const GridFOV_t _global_fov,
                   const CellProbability_t _cell_probability);
    ~Grid();
    void setProbability(PointRAP_t* _data, const double val);
    void computeLikelihood(const std::vector<PolarPose>& pose, PointRAP_t* _data);
    void sensorUpdate(double rate);
    void worldUpdate(PointRAP_t *world_base, double rate);
    void fuse(PointRAP_t* input);
    void normalize(PointRAP_t* _data);
    void output();
};

#endif
