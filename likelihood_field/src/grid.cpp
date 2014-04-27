#include "grid.h"
#include <assert.h>
#include  <stdio.h>

#define _USE_MATH_DEFINES

double toRadian(double t)
{
    return t*M_PI/180;
}

double normalDistribution(const double x, const double u,const double s)
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

void Grid::initGrid(PointRAP_t* _data, const double val)
{
    size_t i,r,c;
    i = 0;
    for(r = 0; r < global_fov.getRowSize(); r++){
        for(c = 0; c < global_fov.getColSize(); c++){
            _data[i].range = global_fov.range.min + global_fov.range.resolution*r;
            _data[i].angle = global_fov.angle.min + global_fov.angle.resolution*c;
            _data[i].probability = val;
            i++;
        }
    }
    //ROS_ASSERT(i == global_fov.getSize()-1);
//    ROS_ASSERT(_data[global_fov.getSize()-1].range == global_fov.range.max);
//    ROS_ASSERT(_data[global_fov.getSize()-1].angle == global_fov.angle.max);
}

void Grid::setUnknownArea()
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(data[i].range >= sensor_fov.range.min &&
                data[i].range <= sensor_fov.range.max &&
                data[i].angle >= sensor_fov.angle.min &&
                data[i].angle <= sensor_fov.angle.max)
            is_known[i] = true;
        else{
            is_known[i] = false;
        }
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


void Grid::setProbability(PointRAP_t* _data, const double val)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        _data[i].probability = val;
    }
}

void Grid::freeProbability(PointRAP_t* _data)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(_data[i].probability < cell_probability.free && is_known[i] == true) _data[i].probability = cell_probability.free;
    }
}

void Grid::unknownProbability(PointRAP_t* _data)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        if(is_known[i] == false) _data[i].probability = cell_probability.unknown;
    }
}

void Grid::computeLikelihood(const std::vector<PolarPose>& poses, PointRAP_t* _data)
{
    if(poses.empty()) return;
    setProbability(_data, 0.0);
    if(poses.size() && !flag) flag = true;

    double mean_range, mean_angle, pr, pt;

// loop through detected human features (e.g. legs, faces, ...)
    for(size_t p = 0; p < poses.size(); p++){
        mean_range = poses.at(p).range - fmod( poses.at(p).range, global_fov.range.resolution) + (global_fov.range.resolution/2);
        mean_angle = poses.at(p).angle - fmod(poses.at(p).angle ,global_fov.angle.resolution) + (global_fov.angle.resolution/2);

        for(size_t j = 0; j < global_fov.getSize(); j++){
            if(is_known[j]){
                pr = normalDistribution((_data[j].range + sensor_fov.range.resolution/2),mean_range,sqrt(global_fov.range.resolution/2));
                pt = normalDistribution((_data[j].angle + sensor_fov.angle.resolution/2),mean_angle,sqrt(global_fov.angle.resolution/2));
                _data[j].probability += pr*pt;
            }
        }
    }
    normalize(_data);
    freeProbability(_data);
    unknownProbability(_data);
}

void Grid::sensorUpdate(double rate)
{
    ROS_ASSERT(rate > 0.0 && rate < 1.0);
    for(size_t i = 0; i < global_fov.getSize(); i++){
        data[i].probability = rate*old_data[i].probability + ((flag) ? (1-rate)*new_data[i].probability : 0.0);
    }
    freeProbability(data);
    unknownProbability(data);
    copyProbability(data, old_data);
}

void Grid::regionNumber(PointRAP_t* _data,
                                        u_int8_t* region_data,
                                        u_int8_t range_region_num,
                                        u_int8_t angle_region_num)
{
    double range_intervals = (global_fov.range.max - global_fov.range.min)/range_region_num;
    double angle_intervals = (global_fov.angle.max - global_fov.angle.min)/angle_region_num;

    for(size_t i = 0; i < global_fov.getSize(); i++){
        for(u_int8_t r = 0; r < range_region_num -1; r++){
            for(u_int8_t t = 0; t < angle_region_num -1; t++){

                region_data[i] = range_region_num*angle_region_num;
                if(_data[i].range > r*range_intervals &&
                        _data[i].range < (r+1)*range_intervals &&
                        _data[i].angle < t*angle_intervals &&
                        _data[i].angle > (t+1)*angle_intervals)
                    region_data[i] = r*angle_region_num + t;
            }
        }
    }
}

void Grid::worldUpdate(PointRAP_t* world_base, double rate)
{
    ROS_ASSERT(rate > 0.0 && rate < 1.0);

    /*
    u_int8_t* region_old_data;
    region_number(old_data, region_old_data, 4, 4);
    u_int8_t* region_world_base;
    region_number(worldGrid_base, region_world_base, 4, 4);
    */

    setProbability(old_data,0.0);
    size_t count, k;
    size_t test = 0;
    for(size_t i = 0; i < global_fov.getSize(); i++){

        count = 0;
        for(k = 0; k < global_fov.getSize(); k++){

            if(old_data[i].distanceAngle(world_base[k].angle) < global_fov.angle.resolution &&
                    old_data[i].distanceRange(world_base[k].range) < global_fov.range.resolution){
                old_data[i].probability += world_base[k].probability;
                count++;
                //ROS_ASSERT(i == k);
               //ROS_INFO("i: %lu    k: %lu", i, k);
            }
        }
        if(count){
            old_data[i].probability = old_data[i].probability/count;
            test++;
        }
    }

    //std::cout << "counter: " << test << "grid size: " << global_fov.getSize() << std::endl;

    for(size_t i = 0; i < global_fov.getSize(); i++){
        data[i].probability = rate*old_data[i].probability +  (1-rate)*new_data[i].probability ;
    }
}


void Grid::fuse(PointRAP_t* input)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        new_data[i].probability = new_data[i].probability*input[i].probability;
    }
}


void Grid::scaleProbability(PointRAP_t* _data, double s)
{
    for(size_t i = 0; i < global_fov.getSize(); i++){
        _data[i].probability = _data[i].probability*s;
    }
}


double Grid::minProbability(PointRAP_t* _data)
{
    double min = _data[0].probability;
    for(size_t i = 1; i < global_fov.getSize(); i++){
        if(_data[i].probability < min)
            min = _data[i].probability;
    }
    return min;
}


double Grid::maxProbability(PointRAP_t* _data)
{
    double max = _data[0].probability;
    for(size_t i = 1; i < global_fov.getSize(); i++){
        if(_data[i].probability > max)
            max = _data[i].probability;
    }
    return max;
}

void Grid::normalize(PointRAP_t* _data)
{
    if(maxProbability(_data) < 0.9) return;
    if(fabs(maxProbability(_data) - minProbability(_data)) < 0.000001) return;
    scaleProbability(_data,(cell_probability.human/maxProbability(_data)));
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







