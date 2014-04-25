#include "grid.h"
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


Grid::Grid(const GridFOV_t _sensor_fov,
                               const GridFOV_t _global_fov,
                               const CellProbability_t _cell_probability):
    global_fov(_global_fov),
    sensor_fov(_sensor_fov),
    cell_probability(_cell_probability),
    flag(false)
{
    init();
}

void Grid::initGrid(PointRAP_t* data, const float val)
{
    size_t i = 0;
    for(size_t r = 0; r < global_fov.getRowSize(); r++){
        for(size_t c = 0; c < global_fov.getColSize(); c++){
            data[i].range = global_fov.range.min + global_fov.range.resolution*r;
            data[i].angle = global_fov.angle.min + global_fov.angle.resolution*c;
            data[i].probability = val;
            i++;
        }
    }
}

void Grid::setUnknownArea()
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(data[i].range >= sensor_fov.range.min &&
                data[i].range <= sensor_fov.range.max &&
                data[i].angle >= sensor_fov.angle.min &&
                data[i].angle <= sensor_fov.angle.max)
            is_known[i] = true;
        else
            is_known[i] = false;
    }
}

void Grid::init()
{
    try
    {
        data = new PointRAP_t[global_fov.getSize()];
        old_data = new PointRAP_t[global_fov.getSize()];
        new_data = new PointRAP_t[global_fov.getSize()];
        is_known = new bool[global_fov.getSize()];
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "in init: bad_alloc caught: " << ba.what() << '\n';
    }

    initGrid(data, cell_probability.unknown);
    initGrid(new_data, cell_probability.unknown);
    initGrid(old_data, cell_probability.unknown);
    setUnknownArea();
}


void Grid::setProbability(PointRAP_t* result, const float val)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        result[i].probability = val;
    }
}

void Grid::freeProbability(PointRAP_t* data)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(data[i].probability < cell_probability.free && is_known[i] == true) data[i].probability = cell_probability.free;
    }
}

void Grid::unknownProbability(PointRAP_t* data)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(is_known[i] == false) data[i].probability = cell_probability.unknown;
    }
}

void Grid::assign(const std::vector<PolarPose>& poses)
{
    if(poses.empty()) return;
    setProbability(new_data, 0.0);
    if(poses.size() && !flag) flag = true;

    float mean_range, mean_angle, pr, pt;

// loop through detected human features (e.g. legs, faces, ...)
    for(size_t p = 0; p < poses.size(); p++){
        mean_range = poses.at(p).range - fmod( poses.at(p).range, global_fov.range.resolution) + (global_fov.range.resolution/2);
        mean_angle = poses.at(p).angle - fmod(poses.at(p).angle ,global_fov.angle.resolution) + (global_fov.angle.resolution/2);

        for(size_t j = 0; j < global_fov.getSize(); j++){
            if(is_known[j]){
                pr = normalDistribution((new_data[j].range + sensor_fov.range.resolution/2),mean_range,sqrt(global_fov.range.resolution/2));
                pt = normalDistribution((new_data[j].angle + sensor_fov.angle.resolution/2),mean_angle,sqrt(global_fov.angle.resolution/2));
                new_data[j].probability += pr*pt;
            }
        }
    }
    normalize(new_data);
    freeProbability(new_data);
    unknownProbability(new_data);
}

void Grid::sensorUpdate(float rate)
{
    assert(rate > 0.0 && rate < 1.0);
    for(size_t i = 0; i < global_fov.getSize(); i++){
        data[i].probability = rate*old_data[i].probability + ((flag) ? (1-rate)*new_data[i].probability : 0.0);
    }
    freeProbability(data);
    unknownProbability(data);
    copyProbability(data, old_data);
}

void Grid::regionNumber(PointRAP_t* data,
                                        u_int8_t* region_data,
                                        u_int8_t range_region_num,
                                        u_int8_t angle_region_num)
{
    float range_intervals = (global_fov.range.max - global_fov.range.min)/range_region_num;
    float angle_intervals = (global_fov.angle.max - global_fov.angle.min)/angle_region_num;

    for(size_t i = 0; i < global_fov.getSize(); i++){
        for(u_int8_t r = 0; r < range_region_num -1; r++){
            for(u_int8_t t = 0; t < angle_region_num -1; t++){

                region_data[i] = range_region_num*angle_region_num;
                if(data[i].range > r*range_intervals &&
                        data[i].range < (r+1)*range_intervals &&
                        data[i].angle < t*angle_intervals &&
                        data[i].angle > (t+1)*angle_intervals)
                    region_data[i] = r*angle_region_num + t;
            }
        }
    }
}

void Grid::worldUpdate(PointRAP_t* world_base, float rate)
{
    assert(rate > 0.0 && rate < 1.0);
//    u_int8_t* region_old_data;
//    region_number(old_data, region_old_data, 4, 4);
//    u_int8_t* region_world_base;
//    region_number(worldGrid_base, region_world_base, 4, 4);

    copyProbability(data, new_data);
    setProbability(old_data,0.0);
    size_t count = 0;
    for(size_t i = 0; i < global_fov.getSize(); i++){
        for(size_t k = 0; k < global_fov.getSize(); k++){
            if(old_data[i].distanceAngle(world_base[k].angle) < global_fov.angle.resolution &&
                    old_data[i].distanceRange(world_base[k].range) < global_fov.range.resolution){
                old_data[i].probability += world_base[k].probability;
                count ++;
            }
        }
        old_data[i].probability = old_data[i].probability/count;
        count = 0;
    }
}


void Grid::fuse(PointRAP_t* input)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        data[i].probability = data[i].probability*input[i].probability;
    }
}


void Grid::scaleProbability(PointRAP_t* data, float s)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        data[i].probability = data[i].probability*s;
    }
}


float Grid::minProbability(PointRAP_t* data)
{
    float min = data[0].probability;
    for(size_t i = 1; i < global_fov.getSize(); i++){
        if(data[i].probability < min)
            min = data[i].probability;
    }
    return min;
}


float Grid::maxProbability(PointRAP_t* data)
{
    float max = data[0].probability;
    for(size_t i = 1; i < global_fov.getSize(); i++){
        if(data[i].probability > max)
            max = data[i].probability;
    }
    return max;
}

void Grid::normalize(PointRAP_t* data)
{
    if(maxProbability(data) < 0.9) return;
    if(fabs(maxProbability(data) - minProbability(data)) < 0.000001) return;
    scaleProbability(data,(cell_probability.human/maxProbability(data)));
}

void Grid::copyProbability(PointRAP_t* source, PointRAP_t* target)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        target[i].probability = source[i].probability;
    }
}


void Grid::output()
{
    std::cout <<"max probability is : " << maxProbability(data) << " " << std::endl;
}

Grid::~Grid()
{
    delete[] data;
    delete[] old_data;
    delete[] new_data;
    delete[] is_known;
}







