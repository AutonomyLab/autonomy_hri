#include "cartesian_grid.h"
#include <assert.h>
#include <stdio.h>

#define _USE_MATH_DEFINES

float normalDistribution(const float x, const float u, const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}

float normalize(const float val,const float x_min, const float x_max, const float range_min, const float range_max)
{
    ROS_ASSERT((x_max - x_min) != 0.0);
    return (range_min + ((range_max - range_min)*(val - x_min) / (x_max - x_min)));
}

float normalize(const float val,const float x_min, const float x_max)
{
    ROS_ASSERT((x_max - x_min) != 0.0);
    return ((val - x_min) / (x_max - x_min));
}


CartesianGrid::CartesianGrid(uint32_t map_size,
                             float_t map_resolution,
                             SensorFOV_t _sensor_fov,
                             CellProbability_t _cell_prob)
{
    ROS_INFO("Constructing an instace of Cartesian Grid.");
    ROS_ASSERT(map_size%2 == 0);

    map.height = map_size;
    map.width = map_size;
    map.resolution = map_resolution;
    map.origin.position.x = (float) map.height*map.resolution / -2.0;
    map.origin.position.y = (float) map.width*map.resolution / -2.0;

    cell_prob = _cell_prob;

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


    posterior.resize(grid_size, cell_prob.free);
    prior.resize(grid_size, cell_prob.free);
    likelihood.resize(grid_size, cell_prob.free);


    setUnknownProbability(posterior, cell_prob.unknown);
    setUnknownProbability(likelihood, cell_prob.unknown);
    setUnknownProbability(prior, cell_prob.unknown);
}


void CartesianGrid::setUnknownProbability(std::vector<float> &data, const float val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}

void CartesianGrid::setFreeProbability(std::vector<float>& data, const float val){
    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}


void CartesianGrid::computeLikelihood(const std::vector<PolarPose>& pose,
                                      std::vector<float> &data,
                                      const float std_range,
                                      const float std_angle)
{
    if(pose.empty())
    {
        setFreeProbability(data, cell_prob.free);
        return;
    }

    //if(poses.size() && !flag) flag = true; [??????]

    float cell_range, cell_angle;
    uint arg_min;
    std::vector<float> dist (pose.size());
    std::vector<float> tmp_lk (grid_size, cell_prob.unknown);

    for(size_t i = 0; i < grid_size; i++){
        cell_range = map.cell_pol_pos.at(i).range;
        cell_angle = map.cell_pol_pos.at(i).angle;

        for(size_t p = 0; p < pose.size(); p++){
            dist[p] = map.cell_pol_pos.at(i).distance(pose.at(p).range, pose.at(p).angle);
        }
        arg_min = std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));

        if(map.cell_inFOV.at(i)){
            tmp_lk[i] = normalDistribution(cell_range, pose.at(arg_min).range, std_range) * normalDistribution(cell_angle, pose.at(arg_min).angle, std_angle);
        }
    }

    float lk_min = *std::min_element(tmp_lk.begin(), tmp_lk.end());
    float lk_max = *std::max_element(tmp_lk.begin(), tmp_lk.end());


    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i))
            data[i] = normalize(tmp_lk[i],lk_min, lk_max, cell_prob.free, cell_prob.human);
    }
//    ROS_INFO("-----------------------------");
//    ROS_INFO("max likelihood: %.2f", maxProbability(data));
//    ROS_INFO("min likelihood: %.2f", minProbability(data));

}

void CartesianGrid::updateGridProbability(std::vector<float>& pr, std::vector<float>& lk, std::vector<float>& po)
{
//    std::vector<float> log_pr(grid_size);
//    std::vector<float> log_po(grid_size);
//    std::vector<float> log_lk(grid_size);

//    toLogOdd(pr, log_pr);
//    toLogOdd(po, log_po);
//    toLogOdd(lk, log_lk);

//    for(size_t i = 0; i < grid_size; i++){
//        if(map.cell_inFOV[i]){
//            log_po[i] = log_pr[i] + log_lk[i];
//        } else{
//            log_po[i] = log_pr[i];
//        }
//    }

//    fromLogOdd(log_pr, pr);
//    fromLogOdd(log_po, po);
//    fromLogOdd(log_lk, lk);


    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV[i]){
    //        po[i] = 0.9 * pr[i] + 0.1 * lk[i]; // LOW PASS Filter
            float den = (lk[i] * pr[i]) + ((1 - lk[i])*(1 - pr[i]));
            float tmp = lk[i] * pr[i] / den;
            po[i] = tmp;
            if(tmp > cell_prob.human)
                po[i] = cell_prob.human;
            else if(tmp < cell_prob.free)
                po[i] = cell_prob.free;
            else
                po[i] = tmp;
        }
    }
    ROS_ASSERT(*std::max_element(po.begin(), po.end()) <= cell_prob.human);
    ROS_ASSERT(*std::min_element(po.begin(), po.end()) >= cell_prob.free);

    pr = po;
    lk = po;
}

void CartesianGrid::toLogOdd(std::vector<float>& pr_data, std::vector<float>& logodd_data)
{
    for(size_t i = 0; i < grid_size; i++){
        ROS_ASSERT((1.0 - pr_data[i]) > 0.0);
        logodd_data[i] = log( ((pr_data[i]) / (1.0 - pr_data[i])));
    }
}

void CartesianGrid::fromLogOdd(std::vector<float>& logodd_data, std::vector<float>& pr_data)
{
    for(size_t i = 0; i < grid_size; i++){
        pr_data[i] = 1.0 - (1.0 / (1.0 + exp(logodd_data[i])));
    }
}


void CartesianGrid::fuse(float *input)
{
//    for(size_t i = 0; i < grid_size; i++){
//        //likelihood[i] = likelihood[i] * input[i];
//    }
}


void CartesianGrid::scaleProbability(float* data, float s)
{
    for(size_t i = 0; i < grid_size; i++){
        data[i] = data[i] * s;
    }
}

CartesianGrid::~CartesianGrid()
{
}







