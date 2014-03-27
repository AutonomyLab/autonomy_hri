#include "lkgrid.h"
#include <assert.h>

#define FREE_CELL_PROBABILITY 0.1

float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
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

void LikelihoodGrid::assign(const std::vector<PolarPose>& poses)
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
                data[r][c] = pr*pt;
            }
        }
    }
}

LikelihoodGrid::~LikelihoodGrid()
{
    for(unsigned int i = 0; i < rows; i++)
        delete[] data[i];

    delete[] data;
}




LikelihoodGrid::LikelihoodGrid(const unsigned int rows, const unsigned int cols):
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
