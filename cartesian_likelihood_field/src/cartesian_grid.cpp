#include "cartesian_grid.h"
#include <assert.h>
#include  <stdio.h>

#define _USE_MATH_DEFINES

float normalDistribution(const float x, const float u, const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}


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
        posterior = new float[grid_size];
        prior = new float[grid_size];
        likelihood = new float[grid_size];
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "in init: bad_alloc caught: " << ba.what() << '\n';
    }
    setGridProbability(posterior, cell_prob.free);
    setGridProbability(likelihood, cell_prob.free);
    setGridProbability(prior, cell_prob.free);

    setUnknownProbability(posterior, cell_prob.unknown);
    setUnknownProbability(likelihood, cell_prob.unknown);
    setUnknownProbability(prior, cell_prob.unknown);
}

void CartesianGrid::setGridProbability(float* data, float val)
{
    std::fill(data, data + grid_size - 1 , val);
}

void CartesianGrid::setUnknownProbability(float* data, float val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}

void CartesianGrid::setFreeProbability(float* data, float val){
    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}


void CartesianGrid::computeLikelihood(const std::vector<PolarPose>& pose,
                                      float* data,
                                      const double std_range,
                                      const double std_angle)
{
    if(pose.empty())
    {
        setFreeProbability(data, cell_prob.free);
        return;
    }

    //if(poses.size() && !flag) flag = true; [??????]
    float* distance;
    float cell_range, cell_angle;
    float nearest_feature;
    distance = new float [pose.size()];

    for(size_t i = 0; i < grid_size; i++){
        cell_range = map.cell_pol_pos.at(i).range;
        cell_angle = map.cell_pol_pos.at(i).angle;

        for(size_t p = 0; p < pose.size(); p++){
            distance[p] = map.cell_pol_pos.at(i).distance(pose.at(p).range, pose.at(p).angle);
        }
        nearest_feature = std::distance(distance, std::min_element(distance, distance + pose.size()));

        if(map.cell_inFOV.at(i)){
            data[i] = normalDistribution(cell_range, pose.at(nearest_feature).range, std_range) * normalDistribution(cell_angle, pose.at(nearest_feature).angle, std_angle);
            //_data[i] = _data[i] * cell_prob.human;
            if(data[i] < 0.1) data[i] = 0.1;
            if(data[i] > 0.9) data[i] = 0.9;
        }
    }

    //ROS_INFO("Max Probability is: %d", maxProbability(data));
    delete[] distance;
}

void CartesianGrid::updateGridProbability(float* pr,
                                     float* lk,
                                     float* po)
{

//    float* log_pr = new float[grid_size];
//    float* log_po = new float[grid_size];
//    float* log_lk = new float[grid_size];

//    toLogOdd(prior, log_pr);
//    toLogOdd(posterior, log_po);
//    toLogOdd(likelihood, log_lk);

//    for(size_t i = 0; i < grid_size; i++){
//        log_po[i] = log_pr[i] + log_lk[i];
//    }

//    fromLogOdd(log_pr, prior);
//    fromLogOdd(log_po, posterior);
//    fromLogOdd(log_lk, likelihood);


    float den;
    for(size_t i = 0; i < grid_size; i++){

//        den = (lk[i] * pr[i]) + (1 - lk[i])*(1 - pr[i]);
//        po[i] = lk[i] * pr[i] / den;

        po[i] = 0.9 * pr[i] + 0.1 * lk[i];
        if(po[i] < 0.1) po[i] = 0.1;
        if(po[i] > 0.9) po[i] = 0.9;
    }

//    int j = 500;
//    ROS_INFO("prior: %f, likelihood: %f, posterior: %f",pr[j], lk[j], po[j]);
//    ROS_INFO("Max Probability prior: %.2f", maxProbability(pr));
//    ROS_INFO("Max Probability likelihood: %.2f", maxProbability(lk));
//    ROS_INFO("Max Probability posterior: %.2f", maxProbability(po));

    copyProbability(po, pr);      // prior = likelihood
    copyProbability(po, lk); // posterior = likelihood // for publishing
}

void CartesianGrid::toLogOdd(float* probability_data, float* logodd_data)
{
    for(size_t i = 0; i < grid_size; i++){
        logodd_data[i] = log((float) ((probability_data[i]) / (100 - probability_data[i])));
    }
}

void CartesianGrid::fromLogOdd(float* logodd_data, float* probability_data)
{
    for(size_t i = 0; i < grid_size; i++){
        probability_data[i] = 100 - (float) (100/(1+exp(logodd_data[i])));
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

void CartesianGrid::copyProbability(float* input, float* output)
{
    for(size_t i = 0; i < grid_size; i++){
        output[i] = input[i];
    }
}



float CartesianGrid::minProbability(float* data)
{
    float min = data[0];
    for(size_t i = 1; i < grid_size; i++){
        if(data[i] < min){
            min = data[i];
        }
    }
    return min;
}


float CartesianGrid::maxProbability(float* data)
{
    float max = data[0];
    for(size_t i = 1; i < grid_size; i++){
        if(data[i] > max){
            max = data[i];
        }
    }
    return max;
}

//void Grid::normalize(PointRAP_t* data)
//{
//    if(maxProbability(data).probability < 0.9) return;
//    if(fabs(maxProbability(data).probability - minProbability(data).probability) < 0.000001) return;
//    scaleProbability(data,(cell_probability.human/maxProbability(data).probability));
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
    delete[] posterior;
    delete[] prior;
    delete[] likelihood;
}







