#include "lkgrid.h"
#include <assert.h>
#include  <stdio.h>

#define _USE_MATH_DEFINES


float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}


LikelihoodGrid::LikelihoodGrid(const GridFOV_t _sensorGridFOV,
                               const GridFOV_t _globalGridFOV,
                   const double _free_cell_probability):
    globalGridFOV(_globalGridFOV),
    sensorGridFOV(_sensorGridFOV),
    free_cell_probability(_free_cell_probability),
    flag(false)
{
    init();
}

void LikelihoodGrid::init()
{
    globalRows = round((globalGridFOV.range.max - globalGridFOV.range.min)/globalGridFOV.range.resolution) + 1;
    globalCols = round((globalGridFOV.angle.max - globalGridFOV.angle.min)/globalGridFOV.angle.resolution) + 1;
    globalGridSize = globalRows * globalCols;

    try
    {
        data = new PointRAP_t[globalGridSize];
        old_data = new PointRAP_t[globalGridSize];
        new_data = new PointRAP_t[globalGridSize];
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "in init: bad_alloc caught: " << ba.what() << '\n';
    }

    size_t i = 0;
    for(unsigned int r = 0; r < globalRows; r++){
        for(unsigned int c = 0; c < globalCols; c++){
            data[i].range = globalGridFOV.range.min + globalGridFOV.range.resolution*r;
            data[i].angle = globalGridFOV.angle.min + globalGridFOV.angle.resolution*c;
            data[i].probability = 0.0;
            new_data[i].range = globalGridFOV.range.min + globalGridFOV.range.resolution*r;
            new_data[i].angle = globalGridFOV.angle.min + globalGridFOV.angle.resolution*c;
            new_data[i].probability = 0.0;
            old_data[i].range = globalGridFOV.range.min + globalGridFOV.range.resolution*r;
            old_data[i].angle = globalGridFOV.angle.min + globalGridFOV.angle.resolution*c;
            old_data[i].probability = 0.0;
            i++;
        }
    }
}


void LikelihoodGrid::set(PointRAP_t* result, const float val)
{
    for(size_t i = 0; i < globalGridSize; i++){
        result[i].probability = val;
    }
}

void LikelihoodGrid::free_lk(float pFree)
{
    for(size_t i = 0; i < globalGridSize; i++){
        if(data[i].probability < pFree) data[i].probability = pFree;
    }
}

void LikelihoodGrid::assign(const std::vector<PolarPose>& poses)
{
    if(poses.empty()) return;
    set(new_data, 0.0);
    if(poses.size() && !flag) flag = true;
    size_t start = 0;
    size_t stop = globalGridSize;


    for(size_t i = 0; i < globalGridSize; i++){

        if(fabs(new_data[i].angle - sensorGridFOV.angle.min) < 0.00001 && fabs(new_data[i].range - sensorGridFOV.range.min)<0.00001){
            start = i;
        }
        if(fabs(new_data[i].angle - sensorGridFOV.angle.max) < 0.00001 && fabs(new_data[i].range - sensorGridFOV.range.max)<0.00001){
            stop = i;
        }
    }

    for(size_t p = 0; p < poses.size(); p++){ // on number of legs
        float correct_angle = poses.at(p).angle - globalGridFOV.angle.min; //TODO: this should be unnecessary!
        float meanDistance = poses.at(p).range - fmod( poses.at(p).range,globalGridFOV.range.resolution) + (globalGridFOV.range.resolution/2);
        float meanAngle = poses.at(p).angle - fmod(fabs(correct_angle),globalGridFOV.angle.resolution) + (globalGridFOV.angle.resolution/2);

        for(size_t j = start; j <= stop; j++){

            float pr = normalDistribution((new_data[j].range + sensorGridFOV.range.resolution/2),meanDistance,sqrt(globalGridFOV.range.resolution/4));
            float pt = normalDistribution((new_data[j].angle + sensorGridFOV.angle.resolution/2),meanAngle,sqrt(globalGridFOV.angle.resolution/4));
            new_data[j].probability += pr*pt;
        }

    }
}

void LikelihoodGrid::update(float rate)
{
    assert(rate > 0.0);
    assert(rate < 1.0);
    for(size_t i = 0; i < globalGridSize; i++){
        if(flag)
            data[i].probability = rate*old_data[i].probability + (1-rate)*new_data[i].probability;
        else
            data[i].probability = rate*old_data[i].probability;
    }
    free_lk(free_cell_probability);
    for(size_t i = 0; i < globalGridSize; i++){
        old_data[i].probability = data[i].probability;
    }
}


void LikelihoodGrid::fuse(PointRAP_t* input)
{
    for(size_t i = 0; i < globalGridSize; i++){
        data[i].probability = data[i].probability*input[i].probability;
    }
}


void LikelihoodGrid::scale(PointRAP_t* data, float s)
{
    for(size_t i = 0; i < globalGridSize; i++){
        data[i].probability = data[i].probability*s;
    }
}


float LikelihoodGrid::min_probability(PointRAP_t* data)
{
    float min = data[0].probability;
    for(size_t i = 1; i < globalGridSize; i++){
        if(data[i].probability < min)
            min = data[i].probability;
    }
    return min;
}


float LikelihoodGrid::max_probability(PointRAP_t* data)
{
    float max = data[0].probability;
    for(size_t i = 1; i < globalGridSize; i++){
        if(data[i].probability > max)
            max = data[i].probability;
    }
    return max;
}

void LikelihoodGrid::normalize()
{
    if(max_probability(data) <= 0.1) return;
    //if(max_probability(data) <= pow(free_cell_probability,3)) return;
    if(fabs(max_probability(data) - min_probability(data)) < 0.000001) return;
    scale(data,(1/max_probability(data)));
}

void LikelihoodGrid::output()
{
    std::cout <<"max probability is : " << max_probability(data) << " " << std::endl;
//    for(size_t i =0; i < globalGridSize; i++){
//        if(data[i].probability > 1){
//            std::cout <<"range:  " << data[i].range << "  angle:  " << data[i].angle <<  "  probablity:  "<< data[i].probability << std::endl;
//        }
//    }
}

LikelihoodGrid::~LikelihoodGrid()
{
    delete[] data;
    delete[] old_data;
    delete[] new_data;
}







