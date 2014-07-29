#include "cartesian_grid.h"
#include <assert.h>
#include <stdio.h>

#define _USE_MATH_DEFINES

double normalDistribution(const double x, const double u, const double s)
{
    return((1.0/(s*sqrt(2.0 * M_PI)))*exp(- 0.5 * pow(x-u,2)/(s * s)));
//    return (1.0/sqrt(2.0 * M_PI)) * exp(-0.5*x*x);
}

double normalize(const double val,const double x_min, const double x_max, const double range_min, const double range_max)
{
    ROS_ASSERT(fabs(x_max - x_min) > 1e-9);
    return (range_min + ((range_max - range_min)*(val - x_min) / (x_max - x_min)));
}

float normalize(const float val,const float x_min, const float x_max)
{
    ROS_ASSERT(fabs(x_max - x_min) > 1e-9);
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
            map.cell_pos_crtsn.push_back(temp_cell_cart_pos);

            temp_cell_pol_pos.fromCart(temp_cell_cart_pos.x, temp_cell_cart_pos.y);
            map.cell_pos_polar.push_back((temp_cell_pol_pos));

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
    predicted_posterior.resize(grid_size, cell_prob.free);
    predicted_likelihood.resize(grid_size, cell_prob.free);

    setUnknownProbability(posterior, cell_prob.unknown);
    setUnknownProbability(likelihood, cell_prob.unknown);
    setUnknownProbability(prior, cell_prob.unknown);
    setUnknownProbability(predicted_posterior, cell_prob.unknown);
    setUnknownProbability(predicted_likelihood, cell_prob.unknown);
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
                                      std::vector<float> &data)
{
    if(pose.empty()){
        setFreeProbability(data, 0.01);
        return;
    }

    float cell_range = -1.0;
    float cell_angle = 2 * M_PI;
    float Gr = 1.0;
    float Ga = 1.0;

    for(size_t i = 0; i < grid_size; i++){
        cell_range = map.cell_pos_polar.at(i).range;
        cell_angle = map.cell_pos_polar.at(i).angle;

        float cell_probability = 0.0;

        if(map.cell_inFOV.at(i)){
            for(size_t p = 0; p < pose.size(); p++){
                if(pose.at(p).range > 0.01)
                    Gr = normalDistribution(cell_range, pose.at(p).range, stdev.range);

                Ga = normalDistribution(cell_angle, pose.at(p).angle, toRadian(stdev.angle));
                cell_probability += Gr * Ga;
            }
            data[i] = cell_probability / pose.size();
        }
    }
}

void CartesianGrid::updateGridProbability(std::vector<float>& pr,
                                          std::vector<float>& lk,
                                          std::vector<float>& po)
{
    std::vector<float> tmp(grid_size, cell_prob.unknown);

    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV[i]){

            float den = (lk[i] * pr[i]) + 0.1 * (1.0 - pr[i]);
            tmp[i] = lk[i] * pr[i] / den;
        }
    }

    float tmp_min = *std::min_element(tmp.begin(), tmp.end());
    float tmp_max = *std::max_element(tmp.begin(), tmp.end());

    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV[i]){
            po[i] = normalize(tmp[i], tmp_min, tmp_max, cell_prob.free, cell_prob.human);
        }
    }

//    pr = po;
//    ROS_ASSERT(*std::max_element(po.begin(), po.end()) <= cell_prob.human);
//    ROS_ASSERT(*std::min_element(po.begin(), po.end()) >= cell_prob.free);
}

void CartesianGrid::bayesOccupancyFilter()
{
    setFreeProbability(likelihood, 0.01);
    setFreeProbability(predicted_likelihood, 0.01);

    computeLikelihood(predicted_polar_array, predicted_likelihood);
    updateGridProbability(prior, predicted_likelihood, predicted_posterior);

    if(diff_time.toSec() < 2.0){
        computeLikelihood(current_polar_array, likelihood);
    }

    updateGridProbability(predicted_posterior, likelihood, posterior);

    prior = posterior;
    last_polar_array = current_polar_array;
}


void CartesianGrid::toLogOdd(std::vector<float>& pr_data, std::vector<float>& logodd_data)
{
    for(size_t i = 0; i < grid_size; i++){
        ROS_ASSERT((1.0 - pr_data[i]) > 0.0);
        logodd_data[i] = log((pr_data[i]) / (1.0 - pr_data[i]));
    }
}

void CartesianGrid::fromLogOdd(std::vector<float>& logodd_data, std::vector<float>& pr_data)
{
    for(size_t i = 0; i < grid_size; i++){
        pr_data[i] = 1.0 - (1.0 / (1.0 + exp(logodd_data[i])));
    }
}


void CartesianGrid::fuse(const std::vector<float> data_1,
                         const std::vector<float> data_2,
                         const std::vector<float> data_3,
                         bool multiply)
{
    if(multiply){
        setFreeProbability(posterior, 1.0);
        setUnknownProbability(posterior, 1.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) * data_2.at(i) * data_3.at(i));
        }
    } else {
        setFreeProbability(posterior, 0.0);
        setUnknownProbability(posterior, 0.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) + data_2.at(i) + data_3.at(i))/3.0;
        }
    }
}


void CartesianGrid::scaleProbability(float* data, float s)
{
    for(size_t i = 0; i < grid_size; i++){
        data[i] = data[i] * s;
    }
}

void CartesianGrid::getPose(geometry_msgs::PoseArray& crtsn_array)
{
    if(!current_polar_array.empty()) current_polar_array.clear();
    PolarPose polar_pose_point;

    for(size_t i = 0; i < crtsn_array.poses.size(); i++){
        polar_pose_point.fromCart(crtsn_array.poses.at(i).position.x, crtsn_array.poses.at(i).position.y);
        current_polar_array.push_back(polar_pose_point);
    }
}

void CartesianGrid::getPose(const autonomy_human::raw_detectionsConstPtr torso_img)
{

    if(!current_polar_array.empty()) current_polar_array.clear();
    PolarPose torso_polar_pose;
    torso_polar_pose.range = -1.0;
    torso_polar_pose.angle = 2 * M_PI;
    geometry_msgs::Point torso_position_in_image;
    double image_width = 640.0;
    double estimated_range = 4.24;
    double estimated_height = 98;
//    double average_person_height = 1.69; //meter
    double camera_fov = 65.0; // degree

    for(size_t i = 0; i < torso_img->detections.size(); i++){
        torso_position_in_image.x = torso_img->detections.at(i).x_offset + 0.5 * torso_img->detections.at(i).width;
//        torso_position_in_image.x = torso_img->faceROI.x_offset + 0.5 * torso_img->faceROI.width;
        torso_polar_pose.angle = atan((image_width/2.0-torso_position_in_image.x)* tan(toRadian(camera_fov/2.0)) * 2.0 / image_width) ;
//        torso_polar_pose.range = estimated_range * torso_img->faceROI.height / estimated_height;
        torso_polar_pose.range = estimated_range * torso_img->detections.at(i).height / estimated_height;
        current_polar_array.push_back(torso_polar_pose);
    }
}

void CartesianGrid::getPose(const hark_msgs::HarkSourceConstPtr& sound_src)
{

    if(!current_polar_array.empty()) current_polar_array.clear();
    PolarPose sound_src_polar;
    sound_src_polar.range = -1.0;
    sound_src_polar.angle = 2 * M_PI;

    for(size_t i = 0; i < sound_src->src.size(); i++){
        if(fabs(sound_src->src[i].y) > 0.0){
            sound_src_polar.angle = toRadian(sound_src->src[i].azimuth);
//            sound_src_polar.range = sqrt(pow(sound_src->src[i].x*2,2) + pow(sound_src->src[i].y*2,2));
            current_polar_array.push_back(sound_src_polar);
        }
    }
}

void CartesianGrid::updateVelocity(double robot_linear_velocity,
                                   double robot_angular_velocity,
                                   double last_polar_range,
                                   double last_polar_angle)
{
    angular_velocity = (robot_linear_velocity * sin(last_polar_angle) /
                            last_polar_range) - robot_angular_velocity;

    linear_velocity = -robot_linear_velocity * cos(last_polar_angle);
}

void CartesianGrid::predict(double robot_linear_velocity, double robot_angular_velocity)
{
    if(!predicted_polar_array.empty()) predicted_polar_array.clear();
    if(last_polar_array.empty()) return;

    diff_time = ros::Time::now() - pre_time;

    predicted_polar_array = last_polar_array;

    for(size_t p = 0 ; p < last_polar_array.size(); p++){

        updateVelocity(robot_linear_velocity, robot_angular_velocity,
                                 last_polar_array.at(p).range,
                                 last_polar_array.at(p).angle);

        predicted_polar_array.at(p).range = linear_velocity * diff_time.toSec() + last_polar_array.at(p).range;
        predicted_polar_array.at(p).angle = angular_velocity * diff_time.toSec() + last_polar_array.at(p).angle;

    }
    pre_time = ros::Time::now();
}

void CartesianGrid::polar2Crtsn(std::vector<PolarPose>& polar_array,
                 geometry_msgs::PoseArray &crtsn_array)
{
    geometry_msgs::Pose crtsn_pose;

    crtsn_array.header.frame_id = "base_footprint";
    crtsn_array.header.stamp = ros::Time::now();

    if(!crtsn_array.poses.empty()) crtsn_array.poses.clear();

    for(size_t p = 0 ; p < polar_array.size(); p++){
        polar_array.at(p).toCart(crtsn_pose.position.x, crtsn_pose.position.y);
        crtsn_pose.position.z = 0.0;
        crtsn_array.poses.push_back(crtsn_pose);
    }
}

CartesianGrid::~CartesianGrid()
{
}







