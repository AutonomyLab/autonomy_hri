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

double normalize(const double val,const double x_min, const double x_max)
{
    ROS_ASSERT(fabs(x_max - x_min) > 1e-9);
    return ((val - x_min) / (x_max - x_min));
}


CartesianGrid::CartesianGrid(uint32_t map_size,
                             double_t map_resolution,
                             SensorFOV_t _sensor_fov,
                             CellProbability_t _cell_prob,
                             double target_detection_probability,
                             double false_positive_probability)
{
    ROS_INFO("Constructing an instace of Cartesian Grid.");
    ROS_ASSERT(map_size%2 == 0);

    map.height = map_size; // 80
    map.width = map_size; // 80
    map.resolution = map_resolution; // 0.5
    map.origin.position.x = (double) map.height*map.resolution / -2.0;
    map.origin.position.y = (double) map.width*map.resolution / -2.0;

    cell_prob = _cell_prob;
    target_detection_prob = target_detection_probability;
    false_positive_prob = false_positive_probability;

    grid_size = map.height * map.width; // 1600
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

    last_highest_lm.index = 0;
    last_highest_lm.tracking = false;
    last_highest_lm.counter = 0;
    geometry_msgs::Point cell_crtsn_position;
    PolarPose cell_polar_position;
    size_t i,r,c;
    i = 0;

//    8 5 2
//    7 4 1
//    6 3 0

    for(c = 0; c < map.width; c++){
        for(r = 0; r < map.height; r++){

            cell_crtsn_position.x = ( x.min + ( map.resolution / 2.0 )) + r * map.resolution;
            cell_crtsn_position.y = ( y.min + ( map.resolution / 2.0 )) + c * map.resolution;
            cell_crtsn_position.z = 0.0;
            map.cell_crtsn.push_back(cell_crtsn_position);

            cell_polar_position.fromCart(cell_crtsn_position.x, cell_crtsn_position.y);
            map.cell_polar.push_back((cell_polar_position));

            if( cell_polar_position.range > sensor_fov.range.min &&
                    cell_polar_position.range < sensor_fov.range.max &&
                    cell_polar_position.angle > sensor_fov.angle.min &&
                    cell_polar_position.angle < sensor_fov.angle.max){

                map.cell_inFOV.push_back(true);
            }else{
                map.cell_inFOV.push_back(false);
            }
            i++;
        }
    }

    ROS_ASSERT( i == grid_size );
    ROS_ASSERT( cell_crtsn_position.x > -x.max ); // The center of last cell is not out of grid
    ROS_ASSERT( cell_crtsn_position.y > -y.max );

//    det_prob = 0.9;
//    false_pos_prob = 0.01;

    posterior.resize(grid_size, cell_prob.free);
    prior.resize(grid_size, cell_prob.free);
    true_likelihood.resize(grid_size, cell_prob.free);
    false_likelihood.resize(grid_size, cell_prob.free);

    predicted_posterior.resize(grid_size, cell_prob.free);
    pred_true_likelihood.resize(grid_size, cell_prob.free);
    pred_false_likelihood.resize(grid_size, cell_prob.free);

    setOutFOVProbability(posterior, cell_prob.unknown);
    setOutFOVProbability(true_likelihood, cell_prob.unknown);
    setOutFOVProbability(false_likelihood, cell_prob.unknown);

    setOutFOVProbability(prior, cell_prob.unknown);
    setOutFOVProbability(predicted_posterior, cell_prob.unknown);
    setOutFOVProbability(pred_true_likelihood, cell_prob.unknown);
    setOutFOVProbability(pred_false_likelihood, cell_prob.unknown);

}


void CartesianGrid::setOutFOVProbability(std::vector<double> &data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}

void CartesianGrid::setInFOVProbability(std::vector<double>& data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i)){
            data[i] = val;
        }
    }
}

void CartesianGrid::predictLikelihood(const std::vector<PolarPose>& pose,
                            std::vector<double> &lk_true,
                            std::vector<double> &lk_false)
{
    double lk_det ;
    double lk_mis ;
    double cell_probability = 0.0;
    double cell_range = -1.0;
    double cell_angle = 2 * M_PI;
    double Gr = 1.0;
    double Ga = 1.0;

    for(size_t i = 0; i < grid_size; i++){

        cell_range = map.cell_polar.at(i).range;
        cell_angle = map.cell_polar.at(i).angle;
        lk_det = cell_prob.unknown;
        lk_mis = cell_prob.unknown;

        cell_probability = 0.0;
        lk_det = 0.25;
        lk_mis = 0.08;

        if(pose.empty()) lk_mis = 0.8;
        else{

            for(size_t p = 0; p < pose.size(); p++){

                Gr = pose.at(p).range < 0.01 ? 1.0 : normalDistribution(cell_range, pose.at(p).range, stdev.range);
                Ga = normalDistribution(cell_angle, pose.at(p).angle, toRadian(stdev.angle));
                cell_probability += Gr * Ga;
            }

            lk_det = (((cell_probability)/(pose.size()) < cell_prob.free) ? cell_prob.free : (cell_probability)/(pose.size()));
        }

        lk_true[i] = (lk_det * target_detection_prob) + (lk_mis * (1.0 - target_detection_prob));
        lk_false[i] = (lk_det * false_positive_prob) + (lk_mis * (1.0 - false_positive_prob));
    }
}

void CartesianGrid::computeLikelihood(const std::vector<PolarPose>& pose,
                                      std::vector<double> &lk_true,
                                      std::vector<double> &lk_false)
{
    double lk_det ;
    double lk_mis ;
    double cell_probability = 0.0;
    double cell_range = -1.0;
    double cell_angle = 2 * M_PI;
    double Gr = 1.0;
    double Ga = 1.0;

    for(size_t i = 0; i < grid_size; i++){

        cell_range = map.cell_polar.at(i).range;
        cell_angle = map.cell_polar.at(i).angle;
        lk_det = cell_prob.unknown;
        lk_mis = cell_prob.unknown;
        cell_probability = 0.0;
//        lk_det = 0.25;
//        lk_mis = 0.08;

        if(map.cell_inFOV.at(i)){
            lk_det = 0.25;
            lk_mis = 0.08;
            if(pose.empty()) lk_mis = 0.8;
            else{

                for(size_t p = 0; p < pose.size(); p++){

                    Gr = pose.at(p).range < 0.01 ? 1.0 : normalDistribution(cell_range, pose.at(p).range, stdev.range);
                    Ga = normalDistribution(cell_angle, pose.at(p).angle, toRadian(stdev.angle));
                    cell_probability += Gr * Ga;
                }

                lk_det = (((cell_probability)/(pose.size()) < cell_prob.free) ? cell_prob.free : (cell_probability)/(pose.size()));
            }
        }
        lk_true[i] = (lk_det * target_detection_prob) + (lk_mis * (1.0 - target_detection_prob));
        lk_false[i] = (lk_det * false_positive_prob) + (lk_mis * (1.0 - false_positive_prob));
    }
}

void CartesianGrid::updateGridProbability(std::vector<double>& pr,
                                          std::vector<double>& true_lk,
                                          std::vector<double>& false_lk,
                                          std::vector<double>& po)
{
//    std::vector<double> tmp(grid_size, cell_prob.unknown);
    double den = 1.0;
    double tmp_po;
    for(size_t i = 0; i < grid_size; i++){

        den = (true_lk[i] * pr[i]) + (false_lk[i] * (1.0 - pr[i]));
        tmp_po = (true_lk[i] * pr[i]) / den;
        po[i] = tmp_po;

        if(tmp_po > cell_prob.human){
            po[i] = tmp_po * cell_prob.human;
        } else if (tmp_po < cell_prob.free) {
            po[i] = cell_prob.free;
        }

        if((!map.cell_inFOV.at(i)) && (po[i]<cell_prob.unknown )){
            po[i] = cell_prob.unknown;
        }
    }
}

void CartesianGrid::bayesOccupancyFilter()
{
    if((angular_velocity > 0.01) || (linear_velocity > 0.01)){
            predictLikelihood(polar_array.predicted, pred_true_likelihood, pred_false_likelihood);
            updateGridProbability(prior, pred_true_likelihood, pred_false_likelihood, predicted_posterior);
    }else{
        predicted_posterior = prior;
    }

    computeLikelihood(polar_array.current, true_likelihood, false_likelihood);
    updateGridProbability(predicted_posterior, true_likelihood, false_likelihood, posterior);

    prior = posterior;
    polar_array.past = polar_array.current;

    max_probability = posterior.at(maxProbCellNum());
}


void CartesianGrid::fuse(const std::vector<double> data_1,
                         const std::vector<double> data_2,
                         const std::vector<double> data_3,
                         bool multiply)
{
    if(multiply){
        setInFOVProbability(posterior, 1.0);
        setOutFOVProbability(posterior, 1.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) * data_2.at(i) * data_3.at(i));
        }
    } else {
        setInFOVProbability(posterior, 0.0);
        setOutFOVProbability(posterior, 0.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) + data_2.at(i) + data_3.at(i))/3.0;
        }
    }
    double x_min = *std::min_element(posterior.begin(), posterior.end());
    double x_max = *std::max_element(posterior.begin(), posterior.end());

//    for(size_t i = 0; i < posterior.size(); i++){
//        normalize(posterior.at(i),x_min, x_max, cell_prob.free, cell_prob.human);
//    }
}


void CartesianGrid::getPose(geometry_msgs::PoseArray& crtsn_array)
{
    polar_array.current.clear();
    PolarPose polar_pose_point;
//    uint num_features = crtsn_array.poses.size();
    for(size_t i = 0; i < crtsn_array.poses.size(); i++){
        polar_pose_point.fromCart(crtsn_array.poses.at(i).position.x, crtsn_array.poses.at(i).position.y);
        polar_array.current.push_back(polar_pose_point);
    }
}

void CartesianGrid::getPose(const autonomy_human::raw_detectionsConstPtr torso_img)
{
    polar_array.current.clear();
    PolarPose torso_polar_pose;
    torso_polar_pose.range = -1.0;
    torso_polar_pose.angle = 2 * M_PI;
    geometry_msgs::Point torso_position_in_image;
    double image_width = 640.0;
//    double estimated_range = 4.0;
//    double estimated_height = 100.0;
    double camera_fov = 65.0; // degree
    //(2.4,280),(3,250),(4,210),(5,180)
    // 0.0000952381 x^2-0.0696716 x+14.4483
    for(size_t i = 0; i < torso_img->detections.size(); i++){
        double h = torso_img->detections.at(i).height;
        torso_position_in_image.x = torso_img->detections.at(i).x_offset + 0.5 * torso_img->detections.at(i).width;
        torso_polar_pose.angle = atan((image_width/2.0-torso_position_in_image.x)* tan(toRadian(camera_fov/2.0)) * 2.0 / image_width) ;
//        torso_polar_pose.range = estimated_range * torso_img->detections.at(i).height / estimated_height;
        torso_polar_pose.range = 0.0000952381*h*h-0.0696716*h+14.4483;
        polar_array.current.push_back(torso_polar_pose);
    }
}

void CartesianGrid::getPose(const hark_msgs::HarkSourceConstPtr& sound_src)
{
    polar_array.current.clear();
    PolarPose sound_src_polar;
    sound_src_polar.range = -1.0;
    sound_src_polar.angle = 2 * M_PI;
    for(size_t i = 0; i < sound_src->src.size(); i++){
        if(fabs(sound_src->src[i].y) > 0.0){
            sound_src_polar.angle = toRadian(sound_src->src[i].azimuth);
//            sound_src_polar.range = sqrt(pow(sound_src->src[i].x*2,2) + pow(sound_src->src[i].y*2,2));
            polar_array.current.push_back(sound_src_polar);
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
    polar_array.predicted.clear();
    if(polar_array.past.empty()) return;

    diff_time = ros::Time::now() - past_time;

    polar_array.predicted = polar_array.past;

    for(size_t p = 0 ; p < polar_array.past.size(); p++){

        updateVelocity(robot_linear_velocity, robot_angular_velocity,
                                 polar_array.past.at(p).range,
                                 polar_array.past.at(p).angle);

        polar_array.predicted.at(p).range = linear_velocity * diff_time.toSec() + polar_array.past.at(p).range;
        polar_array.predicted.at(p).angle = angular_velocity * diff_time.toSec() + polar_array.past.at(p).angle;

    }
    past_time = ros::Time::now();
}

void CartesianGrid::polar2Crtsn(std::vector<PolarPose>& polar_array,
                 geometry_msgs::PoseArray &crtsn_array)
{
    geometry_msgs::Pose crtsn_pose;

    crtsn_array.header.frame_id = "base_footprint";
    crtsn_array.header.stamp = ros::Time::now();

    crtsn_array.poses.clear();

    for(size_t p = 0 ; p < polar_array.size(); p++){
        polar_array.at(p).toCart(crtsn_pose.position.x, crtsn_pose.position.y);
        crtsn_pose.position.z = 0.0;
        crtsn_array.poses.push_back(crtsn_pose);
    }
}

size_t CartesianGrid::maxProbCellNum()
{
    double max = posterior.at(0);
    size_t _cell_num = 0;

    for(size_t i = 1; i < grid_size; i++){
        _cell_num = (max < posterior.at(i)) ? i : _cell_num;
    }
    return _cell_num;
}


void CartesianGrid::getLocalMaximas()
{
    new_lm.clear();
    LocalMaxima_t tmp_new;
    int x = 0;
    tmp_new.counter = 1;
    tmp_new.tracking = false;
    std::vector<int8_t> filter_grid(grid_size, -1);
    std::vector<size_t> neighbors;
    int8_t ss = 3; //searching size : ss = 3 --> ~1.00 meter around the person
    int8_t row_shift = 1;
    int8_t col_shift = map.height;
    bool lm = false;
    double dist_threshold = 0.5;

    for(size_t k = 0; k < grid_size; k++){
        non_local_maxima:
        tmp_new.index = k;
        lm = false;
        uint8_t col = k / map.height;
        uint8_t row = k % map.height;

        if ((posterior.at(k) <= posterior.at(0)) ||(filter_grid.at(k) == 0)) filter_grid.at(k) = 0;
        else {
            filter_grid.at(k) = 0;
            for(int8_t c = -ss; c <= ss; c++){
                int search_col = c + col;
                if(search_col >= map.height || search_col < 0) continue;

                for(int8_t r = -ss; r <= ss; r++){
                    int search_row = r + row;
                    if(search_row > map.height || search_row < 0) continue;

                    x = (search_row * row_shift) + (search_col * col_shift) ;
                    if(x < 0 || x == k || x >= grid_size) continue;  // make sure the index is valid

                    if(posterior.at(k) >= posterior.at(x)){ // if the cell(k) has higher prob. than its neighbor(x)
                        lm = true;
                        neighbors.push_back(x);
                    } else goto non_local_maxima;
                }
            }

            if(lm){
//                if (new_lm.empty()) new_lm.push_back(tmp_new);
//                else{
//                    for(size_t i = 0; i < new_lm.size(); i++){
//                        double dist = cellsDistance(tmp_new.index, new_lm.at(i).index);
//                        if(dist <= 1.0){
//                            if(posterior.at(new_lm.at(i).index) <= posterior.at(tmp_new.index)) new_lm.at(i).index = tmp_new.index;
//                        }else{
//                            new_lm.push_back(tmp_new);
//                        }
//                    }
//                }

                new_lm.push_back(tmp_new);
                filter_grid.at(k) = 1;
                for(size_t n = 0; n < neighbors.size(); n++){
                    filter_grid.at(neighbors.at(n)) = 0;
                }
            }
        }
        neighbors.clear();
    }

    //---------------------------
}

void CartesianGrid::trackLocalMaximas()
{
    LocalMaxima_t tmp_match;
    matched_lm = old_lm;
    main_lm.clear();
    int8_t counter_threshold = 5;

    if(old_lm.empty()){
        old_lm = new_lm;
        return;
    }

    std::vector<bool> find_match(new_lm.size(),false);
    double dist_threshold = 0.5;
    double prob_threshold = 0.1;

    for(size_t i = 0; i < new_lm.size(); i++){

        for(size_t j = 0; j < old_lm.size(); j++){

            if(cellsDistance(new_lm.at(i).index, old_lm.at(j).index) < dist_threshold && !find_match.at(i)){
                matched_lm.at(j).index = ((posterior.at(new_lm.at(i).index) - posterior.at(matched_lm.at(j).index)) > prob_threshold) ? new_lm.at(i).index : matched_lm.at(j).index;
//                matched_lm.at(j).index = new_lm.at(i).index;
                matched_lm.at(j).counter = old_lm.at(j).counter + 1;
                find_match.at(i) = true;
            }
        }

        if(!find_match.at(i)){
            tmp_match.index = new_lm.at(i).index;
            tmp_match.counter = 1;
            tmp_match.tracking = false;
            matched_lm.push_back(tmp_match);
        }
    }
    ROS_ASSERT(old_lm.size() <= matched_lm.size());

    for(size_t k = 0; k < old_lm.size(); k++){
        if(matched_lm.at(k).counter == old_lm.at(k).counter){
            matched_lm.at(k).counter = old_lm.at(k).counter - 1;
        }
    }

    old_lm.clear();

    for(size_t a = 0; a < matched_lm.size(); a++){
        size_t index = matched_lm.at(a).index;

        if(posterior.at(index) < posterior.at(0)) matched_lm.at(a).counter = -counter_threshold-1;

        if(matched_lm.at(a).counter > counter_threshold){
            matched_lm.at(a).counter = counter_threshold +1;
            if(!matched_lm.at(a).tracking) matched_lm.at(a).tracking = true;
        }
        if(matched_lm.at(a).counter <= 0){
            matched_lm.at(a).tracking = false;
        } else {
            old_lm.push_back(matched_lm.at(a));
        }
        if(matched_lm.at(a).tracking == true )
            main_lm.push_back(matched_lm.at(a));
    }

//    main_lm = new_lm;

    local_maxima_poses.poses.clear();
    geometry_msgs::Pose tmp_pose;
    for(size_t j = 0; j < main_lm.size(); j++){
        uint index = main_lm.at(j).index;
        tmp_pose.position.x = map.cell_crtsn.at(index).x;
        tmp_pose.position.y = map.cell_crtsn.at(index).y;
        tmp_pose.position.z = 0.0;
        local_maxima_poses.poses.push_back(tmp_pose);
    }
    local_maxima_poses.header.frame_id = "base_footprint";
    local_maxima_poses.header.stamp = ros::Time::now();
}

void CartesianGrid::trackMaxProbability()
{
    int8_t counter_threshold = 5;
    double dist_threshold = 0.5;

    if(main_lm.empty()){
        LocalMaxima_t look_behind;
        look_behind.index = 0;
        main_lm.push_back(look_behind);
        return;
    }
    uint max_index = main_lm.at(0).index;
    for(size_t i = 0; i < main_lm.size(); i++){
        uint index = main_lm.at(i).index;
        if(posterior.at(max_index) <= posterior.at(index)){
            max_index = index;
        }
    }

    if(cellsDistance(max_index, last_highest_lm.index) < dist_threshold){
        last_highest_lm.index = (posterior.at(max_index) < posterior.at(last_highest_lm.index)) ? last_highest_lm.index : max_index;
        last_highest_lm.counter = last_highest_lm.counter + 1;
    }else{
        last_highest_lm.counter = last_highest_lm.counter -1;
    }

    if(last_highest_lm.counter > counter_threshold){
        last_highest_lm.tracking = true;
        last_highest_lm.counter = counter_threshold +1;
    }
    if(last_highest_lm.counter < 0){
        last_highest_lm.index = max_index;
        last_highest_lm.tracking = false;
        last_highest_lm.counter = 0;
    }
    if(last_highest_lm.tracking){
        highest_prob_point.point.x = map.cell_crtsn.at(last_highest_lm.index).x;
        highest_prob_point.point.y = map.cell_crtsn.at(last_highest_lm.index).y;
        highest_prob_point.point.z = 0.0;
    }
    highest_prob_point.header.frame_id = "base_footprint";
    highest_prob_point.header.stamp = ros::Time::now();
}

void CartesianGrid::updateLocalMaximas()
{
    getLocalMaximas();
    trackLocalMaximas();
    trackMaxProbability();
}

double CartesianGrid::cellsDistance(size_t c1, size_t c2)
{
    double diff_x = map.cell_crtsn.at(c1).x - map.cell_crtsn.at(c2).x;
    double diff_y = map.cell_crtsn.at(c1).y - map.cell_crtsn.at(c2).y;
    return (sqrt(diff_x*diff_x + diff_y*diff_y));
}


CartesianGrid::~CartesianGrid()
{}







