#ifndef LKGRID_H
#define LKGRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "polarcord.h"



/*Geometry_msgs/PoseArray.msg
 * std_msgs/Header header
 * geometry_msgs/Pose[] poses
 *      geometry_msgs/Pose.msg
 *          geometry_msgs/Point position (in ros coordinate frame convention)
 *          geometry_msgs/Quaternion orientation
 * /*/


template <class T>
class Array2D
{
public:
    const unsigned int rows;
    const unsigned int cols;
    T** arr;
private:
    void init(const float val);

public:
    Array2D(const unsigned int r,
            const unsigned int c);
    Array2D(const unsigned int r,
            const unsigned int c,
            const T val);
    void resize(const unsigned int r,
                const unsigned int c);
    ~Array2D();
};

class LikelihoodGrid
{
public:
    unsigned int rows;
    unsigned int cols;
    float rangeMin;
    float rangeMax;
    float rangeRes;
    float angleMin;
    float angleMax;
    float angleRes;
    double grid_angle_min;
    double grid_angle_max;
    double grid_range_min;
    double grid_range_max;
    double free_cell_probability;
private:

    unsigned int colStart;
    unsigned int colStop;
    unsigned int rowStart;
    unsigned int rowStop;
    void init(const float val);
public:
    float** data;
    float** pastdata;
    float** newdata;
    bool flag;
    LikelihoodGrid(const unsigned int rows,
                   const unsigned int cols,
                   const float rmin,
                   const float rmax,
                   const float rres,
                   const float tmin,
                   const float tmax,
                   const float tres,
                   const float val,
                   const double free_cell_probability);
    ~LikelihoodGrid();
    void set(float** data, const float val);
    void free_lk(float pFree);
    void assign(const std::vector<PolarPose>& pose);
    void update(float rate);
    void fuse(float** input);
    void scale(float** data, float s);
    void normalize();
    float min(float** data);
    float max(float** data);
};



#endif
