#include "lkgrid.h"
#include <assert.h>
#include  <stdio.h>

#define _FREE_CELL_PROBABILITY 0.1
#define _USE_MATH_DEFINES


float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}


template <class T>
Array2D<T>::Array2D(const unsigned int r, const unsigned int c):
    rows(r),
    cols(c),
    arr(0)
{
    init(0.0);
}

template <class T>
Array2D<T>::Array2D(const unsigned int r, const unsigned int c, const T val):
    rows(r),
    cols(c),
    arr(0)
{
    init(val);
}

template <class T>
void Array2D<T>::resize(const unsigned int r, const unsigned int c)
{
    rows = 0;
    cols = 0;
    if(!arr){
        init(0.0);
    }
}

template <class T>
void Array2D<T>::init(const float val)
{
    arr = new T*[rows];
    for(unsigned int i = 0; i < rows; i++){
        arr[i] = new T[cols];
        std::fill(arr[i],arr[i]+cols,val);
    }
}

template <class T>
Array2D<T>::~Array2D()
{
    for(unsigned int i = 0; i < rows; i++)
        delete [] arr[i];

    delete [] arr;
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
    data(0),
    pastdata(0),
    newdata(0),
    flag(false)
{
    init(val);
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
    data(0),
    pastdata(0),
    newdata(0),
    flag(false)
{
    init(0.0);
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
    cols(cols),
    rangeMin(rmin),
    rangeMax(rmax),
    rangeRes(rres),
    angleMin(tmin),
    angleMax(tmax),
    angleRes(tres),
    data(0),
    pastdata(0),
    newdata(0),
    flag(false)
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
    pastdata = new float*[rows];
    for(unsigned int i = 0; i < rows; i++){
        pastdata[i] = new float[cols];
        std::fill(pastdata[i],pastdata[i]+cols,val);
    }
    newdata = new float*[rows];
    for(unsigned int i = 0; i < rows; i++){
        newdata[i] = new float[cols];
        std::fill(newdata[i],newdata[i]+cols,0.0);
    }
}

void LikelihoodGrid::set(float** data, const float val)
{
    for(unsigned int i = 0; i < rows; i++)
        std::fill(data[i],data[i]+cols,val);
}

void LikelihoodGrid::free_lk(float pFree)
{
    for(unsigned int r = 0; r < rows; r++){
        for(unsigned int c = 0; c < cols; c++)
            if(data[r][c] < pFree) data[r][c] = pFree;
    }
}

void LikelihoodGrid::assign(const std::vector<PolarPose>& poses)
{
    set(newdata, 0.0);
    if(poses.size() && !flag) flag = true;
    for(size_t p = 0; p < poses.size(); p++){ // on number of legs
        float correct_angle = poses.at(p).angle - _GRID_ANGLE_MIN;
        float meanDistance = poses.at(p).range - fmod( poses.at(p).range,rangeRes) + (rangeRes/2);
        float meanAngle = fabs(correct_angle) - fmod(fabs(correct_angle),angleRes) + (angleRes/2);

        if(correct_angle < 0)   meanAngle = -meanAngle;

        for(unsigned int r = rowStart; r < rowStop; r++){
            for(unsigned int c = colStart; c < colStop; c++){

                float pr = normalDistribution(rangeRes*(r+0.5),meanDistance,sqrt(0.2));
                float pt = normalDistribution((angleRes*(c+0.5)/20),meanAngle/20,sqrt(0.2));
                newdata[r][c] += pr*pt;
            }
        }
    }
}

void LikelihoodGrid::update(float rate)
{
    assert(rate > 0.0);
    assert(rate < 1.0);
    //for(unsigned int r = rowStart; r < rowStop; r++){
        //for(unsigned int c = colStart; c < colStop; c++){
    for(unsigned int r = 0; r < rows; r++){
        for(unsigned int c = 0; c < cols; c++){
            if(flag) data[r][c] = rate*pastdata[r][c] + (1-rate)*newdata[r][c];
            else data[r][c] = rate*pastdata[r][c];
        }
    }
    free_lk(_FREE_CELL_PROBABILITY);

    for(unsigned int r = rowStart; r < rowStop; r++)
        std::copy(data[r], data[r] + cols, pastdata[r]);
}

void LikelihoodGrid::fuse(float** input)
{
    for(unsigned int r = 0; r < rows; r++){
        for(unsigned int c = 0; c < cols; c++)
            data[r][c] = data[r][c]*input[r][c];
    }
}

void LikelihoodGrid::scale(float** data, float s)
{
    for(unsigned int r = 0; r < rows; r++){
        for(unsigned int c = 0; c < cols; c++)
            data[r][c] = data[r][c]*s;
    }
}

float LikelihoodGrid::min(float** data)
{
    float min = *std::min_element(data[0],data[0]+cols);
    for(unsigned int i = 1; i < rows; i++){
        if(*std::min_element(data[i],data[i]+cols) < min)
            min = *std::min_element(data[i],data[i]+cols);
    }
    return min;
}

float LikelihoodGrid::max(float** data)
{
    float max = *std::max_element(data[0],data[0]+cols);
    for(unsigned int i = 1; i < rows; i++){
        if(*std::max_element(data[i],data[i]+cols) > max)
            max = *std::max_element(data[i],data[i]+cols);
    }
    return max;
}

void LikelihoodGrid::normalize()
{
    if(max(data) <= pow(_FREE_CELL_PROBABILITY,3)) return;
    if(fabs(max(data) - min(data)) < 0.000001) return;
    scale(data,(1/max(data)));
}

LikelihoodGrid::~LikelihoodGrid()
{
    for(unsigned int i = 0; i < rows; i++){
        delete [] data[i];
        delete [] pastdata[i];
        delete [] newdata[i];
    }

    delete [] data;
    delete [] pastdata;
    delete [] newdata;
}







