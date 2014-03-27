#ifndef LKGRID_H
#define LKGRID_H

#include <vector>
#include <cmath>
#include <algorithm>


#define _GRID_ANGLE_MIN -90.0
#define _GRID_ANGLE_MAX 90.0
#define _GRID_ANGLE_RESOLUTION 10.0

#define _GRID_RANGE_MIN 0.0
#define _GRID_RANGE_MAX 8.0
#define _GRID_RANGE_RESOLUTION 0.5

#define FREE_CELL_PROBABILITY 0.1


class PolarPose
{
public:
    float range;
    float angle;
    PolarPose():range(0.0), angle(0.0){;}
    PolarPose(const float r, const float a ):range(r), angle(a){;}
    inline void fromCart(const float x, const float y)
    {
        range = sqrt((x * x) + (y * y));
        angle = atan2(y, x)*180/M_PI;
    }
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
private:

    unsigned int colStart;
    unsigned int colStop;
    unsigned int rowStart;
    unsigned int rowStop;
    void init(const float val);
public:
    //float data[GRID_ROWS][GRID_COLS];// The likelihood [0..100] for convinience
    float** data;

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
    void set(const float val);
    void free_lk(float pFree);
    void assign(const std::vector<PolarPose>& pose);
};

#endif
