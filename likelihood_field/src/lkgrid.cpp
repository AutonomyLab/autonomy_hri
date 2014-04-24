#include "lkgrid.h"
#include <assert.h>
#include  <stdio.h>

#define _USE_MATH_DEFINES

float toRadian(float t)
{
    return t*M_PI/180;
}

float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}


LikelihoodGrid::LikelihoodGrid(const GridFOV_t _sensorGridFOV,
                               const GridFOV_t _globalGridFOV,
                               CellProbability_t _cell_probability):
    globalGridFOV(_globalGridFOV),
    sensorGridFOV(_sensorGridFOV),
    cell_probability(_cell_probability),
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
        isKnown = new bool[globalGridSize];
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
            data[i].probability = 1.0;
            new_data[i].range = globalGridFOV.range.min + globalGridFOV.range.resolution*r;
            new_data[i].angle = globalGridFOV.angle.min + globalGridFOV.angle.resolution*c;
            new_data[i].probability = 1.0;
            old_data[i].range = globalGridFOV.range.min + globalGridFOV.range.resolution*r;
            old_data[i].angle = globalGridFOV.angle.min + globalGridFOV.angle.resolution*c;
            old_data[i].probability = 1.0;
            if(data[i].range >= sensorGridFOV.range.min &&
                    data[i].range <= sensorGridFOV.range.max &&
                    data[i].angle >= sensorGridFOV.angle.min &&
                    data[i].angle <= sensorGridFOV.angle.max)
            {
                isKnown[i] = true;
            } else
            {
                isKnown[i] = false;
            }
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

void LikelihoodGrid::free_lk(PointRAP_t* data)
{
    for(size_t i = 0; i < globalGridSize; i++){
        if(data[i].probability < cell_probability.free && isKnown[i] == true) data[i].probability = cell_probability.free;
    }
}

void LikelihoodGrid::unknown_lk(PointRAP_t* data)
{
    for(size_t i = 0; i < globalGridSize; i++){
        if(isKnown[i] == false) data[i].probability = cell_probability.unknown;
    }
}

void LikelihoodGrid::assign(const std::vector<PolarPose>& poses)
{
    if(poses.empty()) return;
    set(new_data, 0.0);
    if(poses.size() && !flag) flag = true;

    float meanDistance, meanAngle, pr, pt;

// loop through detected human features (e.g. legs, faces, ...)
    for(size_t p = 0; p < poses.size(); p++){
        meanDistance = poses.at(p).range - fmod( poses.at(p).range, globalGridFOV.range.resolution) + (globalGridFOV.range.resolution/2);
        meanAngle = poses.at(p).angle - fmod(poses.at(p).angle ,globalGridFOV.angle.resolution) + (globalGridFOV.angle.resolution/2);

        for(size_t j = 0; j < globalGridSize; j++){
            if(isKnown[j]){
                pr = normalDistribution((new_data[j].range + sensorGridFOV.range.resolution/2),meanDistance,sqrt(globalGridFOV.range.resolution/2));
                pt = normalDistribution((new_data[j].angle + sensorGridFOV.angle.resolution/2),meanAngle,sqrt(globalGridFOV.angle.resolution/2));
                new_data[j].probability += pr*pt;
            }
        }
    }
    normalize(new_data);
    free_lk(new_data);
    unknown_lk(new_data);
}

void LikelihoodGrid::update(float rate)
{
    assert(rate > 0.0 && rate < 1.0);
    for(size_t i = 0; i < globalGridSize; i++){
        data[i].probability = rate*old_data[i].probability + ((flag) ? (1-rate)*new_data[i].probability : 0.0);
    }
    free_lk(data);
    unknown_lk(data);
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

void LikelihoodGrid::normalize(PointRAP_t* data)
{
    if(max_probability(data) < 0.9) return;
    if(fabs(max_probability(data) - min_probability(data)) < 0.000001) return;
    scale(data,(cell_probability.human/max_probability(data)));
}

void LikelihoodGrid::output()
{
    std::cout <<"max probability is : " << max_probability(data) << " " << std::endl;
}

LikelihoodGrid::~LikelihoodGrid()
{
    delete[] data;
    delete[] old_data;
    delete[] new_data;
    delete[] isKnown;
}







