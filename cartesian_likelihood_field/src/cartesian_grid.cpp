#include "cartesian_grid.h"
#include <assert.h>
#include  <stdio.h>

#define _USE_MATH_DEFINES


CartesianGrid::CartesianGrid(uint32_t map_size,
                             float_t map_resolution,
                             SensorFOV_t _sensor_fov,
                             CellProbability_t cell_prob)
{
    ROS_INFO("Constructing an instace of Cartesian Grid.");
    ROS_ASSERT(map_size%2 == 0);

    map.height = map_size;
    map.width = map_size;
    map.resolution = map_resolution;
    map.origin.position.x = (float) map.height*map.resolution / -2.0;
    map.origin.position.y = (float) map.width*map.resolution / -2.0;

    grid_size = map.height * map.width;
    sensor_fov = _sensor_fov;

    x.max = map.height * map.resolution / 2;
    y.max = map.width * map.resolution / 2;
    x.min = -x.max;
    y.min = -y.max;

    ROS_INFO("Map Info: Height: %d      Width:  %d      Resolution: %.2f    origin: (%.2f, %.2f)",
             map.height,
             map.width,
             map.resolution,
             map.origin.position.x,
             map.origin.position.y);

    ROS_INFO("Grid Info: size: %d      x_max:  %.2f      y_max: %.2f",
             grid_size,
             x.max,
             y.max);

    geometry_msgs::Point temp_cell_cart_pos;
    PolarPose temp_cell_pol_pos;
    size_t i,r,c;

    i = 0;
    for(c = 0; c < map.width; c++){
        for(r = 0; r < map.height; r++){

            temp_cell_cart_pos.x = ( x.min + ( map.resolution / 2.0 )) + r * map.resolution;
            temp_cell_cart_pos.y = ( y.min + ( map.resolution / 2.0 )) + c * map.resolution;
            temp_cell_cart_pos.z = 0.0;
            map.cell_cart_pos.push_back(temp_cell_cart_pos);

            temp_cell_pol_pos.fromCart(temp_cell_cart_pos.x, temp_cell_cart_pos.y);
            map.cell_pol_pos.push_back((temp_cell_pol_pos));

            if( temp_cell_pol_pos.range > sensor_fov.range.min &&
                    temp_cell_pol_pos.range < sensor_fov.range.max &&
                    temp_cell_pol_pos.angle > sensor_fov.angle.min &&
                    temp_cell_pol_pos.angle < sensor_fov.angle.max){

                map.cell_inFOV.push_back(true);
            }else{
                map.cell_inFOV.push_back(false);
            }
            i++;
        }
    }

    ROS_ASSERT( i == grid_size );
    ROS_ASSERT( temp_cell_cart_pos.x > -x.max ); // The center of last cell is not out of grid
    ROS_ASSERT( temp_cell_cart_pos.y > -y.max );

    try
    {
        data = new int8_t[grid_size];
        old_data = new int8_t[grid_size];
        new_data = new int8_t[grid_size];
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "in init: bad_alloc caught: " << ba.what() << '\n';
    }
    setGridProbability(data, cell_prob.free);
    setGridProbability(new_data, cell_prob.free);
    setGridProbability(old_data, cell_prob.free);

    setUnknownProbability(data, cell_prob.unknown);
    setUnknownProbability(new_data, cell_prob.unknown);
    setUnknownProbability(old_data, cell_prob.unknown);

}

void CartesianGrid::setGridProbability(int8_t*_data, uint8_t val)
{
    std::fill(_data, _data + grid_size - 1 , val);
}

void CartesianGrid::setUnknownProbability(int8_t* _data, uint8_t val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)){
            _data[i] = val;
        }
    }
}


void CartesianGrid::computeLikelihood(const std::vector<PolarPose>& pose, int8_t* _data, const double std_range,const double std_angle)
{
    if(pose.empty()) return;

    //if(poses.size() && !flag) flag = true; [??????]

    double cell_range, cell_angle;
    double* distance;
    uint8_t nearest_feature;
    distance = new double [pose.size()];

    for(size_t i = 0; i < grid_size; i++){
        cell_range = map.cell_pol_pos.at(i).range;
        cell_angle = map.cell_pol_pos.at(i).angle;

        for(size_t p = 0; p < pose.size(); p++){
            distance[p] = map.cell_pol_pos.at(i).distance(pose.at(p).range, pose.at(p).angle);
        }
        nearest_feature = std::distance(distance, std::min_element(distance, distance + pose.size() - 1));

        if(map.cell_inFOV.at(i)){
            if( fabs(cell_range - pose.at(nearest_feature).range) < std_range &&
                    fabs(cell_angle - pose.at(nearest_feature).angle) < std_angle) {
                _data[i] = 100;
            } else {
                _data[i] = 0;
            }
        }
    }

    delete[] distance;
}

void CartesianGrid::fuse(int8_t* input)
{
    for(size_t i = 0; i < grid_size; i++){
        new_data[i] = new_data[i] * input[i];
    }
}


void CartesianGrid::scaleProbability(int8_t* _data, double s)
{
    for(size_t i = 0; i < grid_size; i++){
        _data[i] = _data[i] * s;
    }
}


int8_t CartesianGrid::minProbability(int8_t* _data)
{
    int8_t min = _data[0];
    for(size_t i = 1; i < grid_size; i++){
        if(_data[i] < min){
            min = _data[i];
        }
    }
    return min;
}


int8_t CartesianGrid::maxProbability(int8_t* _data)
{
    int8_t max = _data[0];
    for(size_t i = 1; i < grid_size; i++){
        if(_data[i] > max){
            max = _data[i];
        }
    }
    return max;
}

//void Grid::normalize(PointRAP_t* _data)
//{
//    if(maxProbability(_data).probability < 0.9) return;
//    if(fabs(maxProbability(_data).probability - minProbability(_data).probability) < 0.000001) return;
//    scaleProbability(_data,(cell_probability.human/maxProbability(_data).probability));
//}

//void Grid::copyProbability(PointRAP_t* source, PointRAP_t* target)
//{
//    for(size_t i = 0; i < global_fov.getSize(); i++){
//        target[i].probability = source[i].probability;
//    }
//}


//PointRAP_t Grid::output()
//{
//ROS_INFO("max probability is : %lf at range: [%lf] angle: [%lf]  ",maxProbability(data).probability,
//         maxProbability(data).range,
//         maxProbability(data).angle);
//}

CartesianGrid::~CartesianGrid()
{
    delete[] data;
    delete[] old_data;
    delete[] new_data;
}







