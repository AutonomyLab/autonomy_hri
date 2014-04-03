#ifndef LKGRID_H
#define LKGRID_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "polarcord.h"


#define _GRID_ANGLE_MIN -90.0
#define _GRID_ANGLE_MAX 90.0
#define _GRID_ANGLE_RESOLUTION 10.0

#define _GRID_RANGE_MIN 0.0
#define _GRID_RANGE_MAX 8.0
#define _GRID_RANGE_RESOLUTION 0.5

#define FREE_CELL_PROBABILITY 0.1


/*Geometry_msgs/PoseArray.msg
 * std_msgs/Header header
 * geometry_msgs/Pose[] poses
 *      geometry_msgs/Pose.msg
 *          geometry_msgs/Point position (in ros coordinate frame convention)
 *          geometry_msgs/Quaternion orientation
 * /*/




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
                   const unsigned int cols);
    LikelihoodGrid(const unsigned int rows,
                   const unsigned int cols,const float val);
    LikelihoodGrid(const unsigned int rows,
                   const unsigned int cols,
                   const float rmin,
                   const float rmax,
                   const float rres,
                   const float tmin,
                   const float tmax,
                   const float tres,
                   const float val);
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
