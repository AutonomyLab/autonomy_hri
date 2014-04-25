#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "polarcord.h"

float toRadian(float t);

struct PointRAP_t{
    float range;
    float angle;
    float probability;
    float distanceRange(float target){return fabs(range - target);}
    float distanceAngle(float target) {return fabs(angle - target);}
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
    PointRAP_t* new_data;
    bool* is_known;
    void init();
    void initGrid(PointRAP_t* data, const float val);
    void setUnknownArea();
    void freeProbability(PointRAP_t* data);
    void unknownProbability(PointRAP_t *data);
    void scaleProbability(PointRAP_t* data, float s);
    float minProbability(PointRAP_t* data);
    float maxProbability(PointRAP_t* data);
    void copyProbability(PointRAP_t* source, PointRAP_t* target);
    void regionNumber(PointRAP_t* data,
                            u_int8_t* region_data,
                            u_int8_t range_region_num,
                            u_int8_t angle_region_num);
public:
    PointRAP_t* data;
    bool flag;
    Grid();
    Grid(const GridFOV_t _sensor_fov,
                   const GridFOV_t _global_fov,
                   const CellProbability_t _cell_probability);
    ~Grid();
    void setProbability(PointRAP_t* data, const float val);
    void assign(const std::vector<PolarPose>& pose);
    void sensorUpdate(float rate);
    void worldUpdate(PointRAP_t *world_base, float rate);
    void fuse(PointRAP_t* input);
    void normalize(PointRAP_t* data);
    void output();
};

#endif
