#ifndef LKGRID_H
#define LKGRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "polarcord.h"


struct PointRAP_t{
    float range;
    float angle;
    float probability;
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
};


class LikelihoodGrid
{
public:
    GridFOV_t globalGridFOV;
    GridFOV_t sensorGridFOV;
    double free_cell_probability;
private:

    unsigned int colStart;
    unsigned int colStop;
    unsigned int rowStart;
    unsigned int rowStop;
    unsigned int colStep;
    unsigned int rowStep;
    unsigned int globalRows;
    unsigned int globalCols;
    PointRAP_t* old_data;
    PointRAP_t* new_data;
    void init();
public:
    PointRAP_t* data;
    size_t globalGridSize;
    bool flag;
    LikelihoodGrid();
    LikelihoodGrid(const GridFOV_t _sensorGridFOV,
                   const GridFOV_t _globalGridFOV,
                   const double _free_cell_probability);
    ~LikelihoodGrid();
    void set(PointRAP_t* data, const float val);
    void free_lk(float pFree);
    void assign(const std::vector<PolarPose>& pose);
    void update(float rate);
    void fuse(PointRAP_t* input);
    void scale(PointRAP_t* data, float s);
    float min_probability(PointRAP_t* data);
    float max_probability(PointRAP_t* data);
    void normalize(float pFree);
    void output();
};

#endif