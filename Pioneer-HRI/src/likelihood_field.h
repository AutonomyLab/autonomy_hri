#ifndef LIKELIHOOD_FIELD_H
#define LIKELIHOOD_FIELD_H

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

float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}

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
    void update(const std::vector<PolarPose>& pose);
};

LikelihoodGrid::LikelihoodGrid(const unsigned int rows,
                               const unsigned int cols):
    rows(rows),
    cols(cols),
    rangeMin(_GRID_RANGE_MIN),
    rangeMax(_GRID_RANGE_MAX),
    rangeRes(_GRID_RANGE_RESOLUTION),
    angleMin(_GRID_ANGLE_MIN),
    angleMax(_GRID_ANGLE_MAX),
    angleRes(_GRID_ANGLE_RESOLUTION),
    data(0)
{
    init(0.0);
}

LikelihoodGrid::LikelihoodGrid(const unsigned int rows,
                               const unsigned int cols,
                               const float val):
    rows(rows),
    cols(cols),
    rangeMin(_GRID_RANGE_MIN),
    rangeMax(_GRID_RANGE_MAX),
    rangeRes(_GRID_RANGE_RESOLUTION),
    angleMin(_GRID_ANGLE_MIN),
    angleMax(_GRID_ANGLE_MAX),
    angleRes(_GRID_ANGLE_RESOLUTION),
    data(0)
{
    init(val);
}


LikelihoodGrid::LikelihoodGrid(const unsigned int rows,
                               const unsigned int cols,
                               const float rmin,
                                 const float rmax,
                                 const float rres,
                                 const float tmin,
                                 const float tmax,
                                 const float tres,
                                 const float val):
    rows(rows),
    cols(cols),rangeMin(rmin), rangeMax(rmax),rangeRes(rres),angleMin(tmin),angleMax(tmax),angleRes(tres), data(0)
{
    init(val);
}

void LikelihoodGrid::init(const float val)
{
    unsigned int rowStep = round((rangeMax - rangeMin)/rangeRes);
    unsigned int colStep = round((angleMax - angleMin)/angleRes);
    colStart = (angleMin - _GRID_ANGLE_MIN)/angleRes;
    colStop = colStart + colStep;
    rowStart = (rangeMin -_GRID_RANGE_MIN)/rangeRes;
    rowStop = rowStart + rowStep;
    assert( rowStart <= rows);
    assert(rowStop <= rows);
    assert( colStart <= cols);
    assert(colStop <= cols);
    data = new float*[rows];
    for(unsigned int i = 0; i < rows; i++){
        data[i] = new float[cols];
        std::fill(data[i],data[i]+cols,val);

    }
}

void LikelihoodGrid::set(const float val)
{
    for(unsigned int i = 0; i < rows; i++){
        std::fill(data[i],data[i]+cols,val);
    }
}

void LikelihoodGrid::free_lk(float pFree)
{
    for(unsigned int r = 0; r < rows; r++){
        for(unsigned int c = 0; c < cols; c++)
            data[r][c] += pFree;
    }
}

void LikelihoodGrid::update(const std::vector<PolarPose>& poses)
{
    set(0.0);

    for(unsigned int p = 0; p < poses.size(); p++){ // on number of legs
        float correct_angle = poses.at(p).angle - _GRID_ANGLE_MIN;
        float meanDistance = poses.at(p).range - fmod( poses.at(p).range,rangeRes) + (rangeRes/2);
        float meanAngle = fabs(correct_angle) - fmod(fabs(correct_angle),angleRes) + (angleRes/2);
        if(correct_angle < 0)   meanAngle = -meanAngle;

        for(unsigned int r = rowStart; r < rowStop; r++){
            for(unsigned int c = colStart; c < colStop; c++){

                float pr = normalDistribution(rangeRes*(r+0.5),meanDistance,sqrt(0.2));
                float pt = normalDistribution((angleRes*(c+0.5)/20),meanAngle/20,sqrt(0.2));

                data[r][c] += pr*pt;
                //if(pr*pt > 0.01)
                  //  ROS_INFO("pr: [%.2f]  pt: [%.2f]  pr*pt [%.2f]  lk:[%.3f]", pr,pt,pr*pt,lk_grid->data[r][c]);
            }
        }
    }
/*
    for(unsigned int rr = 0; rr < GRID_ROWS; rr++){
        for(unsigned int cc = 0; cc < GRID_COLS; cc++){

            if(lk_grid->data[rr][cc] > 0.01)
                ROS_INFO("Likelihood is : [%.2f] for a person in [%.2f] away and in [%.2f] degree"
                         ,lk_grid->data[rr][cc],
                         (rr + 0.5)*_GRID_RANGE_RESOLUTION,
                         (cc + 0.5)*_GRID_ANGLE_RESOLUTION);
        }
    }
    */

}

LikelihoodGrid::~LikelihoodGrid()
{
    for(unsigned int i = 0; i < rows; i++)
        delete[] data[i];

    delete[] data;
}


#endif
