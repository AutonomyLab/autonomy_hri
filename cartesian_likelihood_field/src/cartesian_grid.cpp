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

double maximum(double a, double b, double c)
{
    double max = (a >= b) ? a : b;
    max = (max >= c) ? max : c;
    return max;
}

CartesianGrid::CartesianGrid(uint32_t map_size,
                             SensorFOV_t _sensor_fov,
                             double_t map_resolution,
                             CellProbability_t _cell_probability,
                             double _target_detection_probability,
                             double _false_positive_probability)
{
    ROS_INFO("Constructing an instace of cartesian Grid.");
    ROS_ASSERT(map_size % 2 == 0);

    map.height = map_size; // DEFAULT 80
    map.width = map_size; // DEFAULT 80
    map.resolution = map_resolution; // DEFAULT 0.25
    map.origin.position.x = (double) map.height*map.resolution / -2.0;
    map.origin.position.y = (double) map.width*map.resolution / -2.0;

    cell_probability = _cell_probability;
    TARGET_DETECTION_PROBABILITY = _target_detection_probability;
    FALSE_DETECTION_PROBABILITY = _false_positive_probability;

    grid_size = map.height * map.width; // DEFAULT 1600
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

//    8 5 2
//    7 4 1
//    6 3 0

    Cell_t temp_cell;
    size_t i = 0;

    for(size_t c = 0; c < map.width; c++){
        for(size_t r = 0; r < map.height; r++){

            temp_cell.cartesian.x = x.min + map.resolution / 2.0 + r * map.resolution;
            temp_cell.cartesian.y = y.min + map.resolution / 2.0 + c * map.resolution;
            temp_cell.cartesian.z = 0.0;
            temp_cell.polar.fromCart(temp_cell.cartesian.x, temp_cell.cartesian.y);

            map.cell.push_back(temp_cell);

            if( temp_cell.polar.range > sensor_fov.range.min &&
                    temp_cell.polar.range < sensor_fov.range.max &&
                    temp_cell.polar.angle > sensor_fov.angle.min &&
                    temp_cell.polar.angle < sensor_fov.angle.max){

                map.cell_inFOV.push_back(true);
            }else{
                map.cell_inFOV.push_back(false);
            }
            i++;
        }
    }

    ROS_ASSERT( i == grid_size );
    ROS_ASSERT( temp_cell.cartesian.x > -x.max );
    ROS_ASSERT( temp_cell.cartesian.y > -y.max );

    posterior.resize(grid_size, cell_probability.unknown);
    prior.resize(grid_size, cell_probability.unknown);
    true_likelihood.resize(grid_size, cell_probability.unknown);
    false_likelihood.resize(grid_size, cell_probability.unknown);

    predicted_posterior.resize(grid_size, cell_probability.unknown);
    predicted_true_likelihood.resize(grid_size, cell_probability.unknown);
    predicted_false_likelihood.resize(grid_size, cell_probability.unknown);

    setOutFOVProbability(posterior, cell_probability.unknown);
    setOutFOVProbability(true_likelihood, cell_probability.unknown);
    setOutFOVProbability(false_likelihood, cell_probability.unknown);

    setOutFOVProbability(prior, cell_probability.unknown);
    setOutFOVProbability(predicted_posterior, cell_probability.unknown);
    setOutFOVProbability(predicted_true_likelihood, cell_probability.unknown);
    setOutFOVProbability(predicted_false_likelihood, cell_probability.unknown);
}

void CartesianGrid::setOutFOVProbability(std::vector<double> &data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)) { data[i] = val; }
    }
}

void CartesianGrid::setInFOVProbability(std::vector<double>& data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i)) { data[i] = val; }
    }
}

void CartesianGrid::computeLikelihood(const std::vector<PolarPose>& pose,
                            std::vector<double> &_true_likelihood,
                            std::vector<double> &_false_likelihood)
{
    for(size_t i = 0; i < grid_size; i++){
        double range = map.cell.at(i).polar.range;
        double angle = map.cell.at(i).polar.angle;
        double detection_likelihood = cell_probability.unknown;
        double miss_detection_likelihood = cell_probability.unknown;
//        double miss_detection_likelihood = 1.0 - detection_likelihood;

        double cell_prob = 0.0;

//        detection_likelihood = cell_probability.unknown;
//        miss_detection_likelihood = cell_probability.unknown;

        if(pose.empty()) miss_detection_likelihood = cell_probability.human; // ?????

        else{
            for(size_t p = 0; p < pose.size(); p++){
                double Gr = (pose.at(p).range < 0.01) ? 1.0 : normalDistribution(range, pose.at(p).range, stdev.range);
                double Ga = normalDistribution(angle, pose.at(p).angle, toRadian(stdev.angle));
                cell_prob += Gr * Ga;
            }
            detection_likelihood = (((cell_prob)/(pose.size()) < cell_probability.free) ? cell_probability.free : (cell_prob)/(pose.size()));
//            miss_detection_likelihood = 0.1; /// ????
        }

        _true_likelihood[i] = (detection_likelihood * TARGET_DETECTION_PROBABILITY) + (miss_detection_likelihood * (1.0 - TARGET_DETECTION_PROBABILITY));
        _false_likelihood[i] = (detection_likelihood * FALSE_DETECTION_PROBABILITY) + (miss_detection_likelihood * (1.0 - FALSE_DETECTION_PROBABILITY));
    }
}


void CartesianGrid::updateGridProbability(std::vector<double>& _prior,
                                          const std::vector<double>& _true_likelihood,
                                          const std::vector<double>& _false_likelihood,
                                          std::vector<double>& _posterior)
{
    // POSTERIOR = TARGET LOCALIZATION PROBABILITY

    double normalizer_factor = 1.0;

    for(size_t i = 0; i < grid_size; i++){

        normalizer_factor = (_true_likelihood[i] * _prior[i]) + (_false_likelihood[i] * (1.0 - _prior[i]));
        _posterior[i] = (_true_likelihood[i] * _prior[i]) / normalizer_factor;
        _posterior[i] = (_posterior[i] > cell_probability.human) ? _posterior[i] * cell_probability.human : _posterior[i];
        _posterior[i] = (_posterior[i] < cell_probability.free) ? cell_probability.free : _posterior[i];

        //PROBABILITY IN unknown AREA CANNOT BE LESS THAN unknown PROBABILITY
        if((!map.cell_inFOV.at(i)) && (_posterior[i] < cell_probability.unknown )){
            _posterior[i] = cell_probability.unknown;
        }
    }
}

void CartesianGrid::bayesOccupancyFilter()
{
    // PREDICTION BY MOTION MODEL
    computeLikelihood(polar_array.predicted, predicted_true_likelihood, predicted_false_likelihood);
    updateGridProbability(prior, predicted_true_likelihood, predicted_false_likelihood, predicted_posterior);

    // UPDATE BY OBSERVATION
    computeLikelihood(polar_array.current, true_likelihood, false_likelihood);
    updateGridProbability(predicted_posterior, true_likelihood, false_likelihood, posterior);

    prior = posterior;
    polar_array.past = polar_array.current;
    max_probability = posterior.at(maxProbCellIndex());
}


void CartesianGrid::fuse(const std::vector<double> data_1,
                         const std::vector<double> data_2,
                         const std::vector<double> data_3,
                         bool multiply)
{
    //TODO: FIX THIS
    if(multiply){

        setInFOVProbability(posterior, 1.0);
        setOutFOVProbability(posterior, 1.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) * data_2.at(i) * data_3.at(i));}
    } else {
        setInFOVProbability(posterior, 0.0);
        setOutFOVProbability(posterior, 0.0);
        for(size_t i = 0; i < grid_size; i++){
            posterior.at(i) = (data_1.at(i) + data_2.at(i) + data_3.at(i))/3.0;}
    }
}


void CartesianGrid::getPose(geometry_msgs::PoseArray& crtsn_array)
{
    polar_array.current.clear();
    PolarPose polar_pose_point;
    for(size_t i = 0; i < crtsn_array.poses.size(); i++){
        polar_pose_point.fromCart(crtsn_array.poses.at(i).position.x, crtsn_array.poses.at(i).position.y);
        polar_array.current.push_back(polar_pose_point);
    }
}

void CartesianGrid::getPose(const autonomy_human::raw_detectionsConstPtr torso_img)
{
    polar_array.current.clear();
    PolarPose torso_polar_pose;

    geometry_msgs::Point torso_position_in_image;
    double image_width = 640.0;;
    double camera_fov = 65.0; // degree
    //(2.4,280),(3,250),(4,210),(5,180)
    // 0.0000952381 x^2-0.0696716 x+14.4483

    for(size_t i = 0; i < torso_img->detections.size(); i++){
        torso_polar_pose.range = -1.0;
        torso_polar_pose.angle = 2 * M_PI;

        torso_position_in_image.x = torso_img->detections.at(i).x_offset + 0.5 * torso_img->detections.at(i).width;
        torso_polar_pose.angle = atan((image_width/2.0-torso_position_in_image.x)* tan(toRadian(camera_fov/2.0)) * 2.0 / image_width) ;

        double h = torso_img->detections.at(i).height;
        torso_polar_pose.range = 0.0000952381 * h * h - 0.0696716 * h + 14.4483;
        polar_array.current.push_back(torso_polar_pose);
    }
}

void CartesianGrid::getPose(const hark_msgs::HarkSourceConstPtr& sound_src)
{
    polar_array.current.clear();
    PolarPose sound_src_polar;

    for(size_t i = 0; i < sound_src->src.size(); i++){
        sound_src_polar.range = -1.0;
        sound_src_polar.angle = 2 * M_PI;

        if(fabs(sound_src->src[i].y) > 0.0){
            sound_src_polar.angle = toRadian(sound_src->src[i].azimuth);
//            sound_src_polar.range = sqrt(pow(sound_src->src[i].x*2,2) + pow(sound_src->src[i].y*2,2));
            polar_array.current.push_back(sound_src_polar);
        }
    }
}

void CartesianGrid::predict(const Velocity_t _robot_velocity)
{
    polar_array.predicted.clear();
    polar_array.predicted = polar_array.past;

    velocity.linear = -_robot_velocity.linear;
    velocity.angular = -_robot_velocity.angular;

    for(size_t p = 0 ; p < polar_array.past.size(); p++){
        polar_array.predicted.at(p).range = velocity.linear * diff_time.toSec() + polar_array.past.at(p).range;
        polar_array.predicted.at(p).angle = velocity.angular * diff_time.toSec() + polar_array.past.at(p).angle;
    }
}

size_t CartesianGrid::predictHighestProbability(size_t index)
{
    PolarPose pose1, pose2;
    double x, y;

    pose1.range = map.cell.at(index).polar.range;
    pose1.angle = map.cell.at(index).polar.angle;

    pose2.range = velocity.linear * diff_time.toSec() + pose1.range;
    pose2.angle = velocity.angular * diff_time.toSec() + pose1.angle;
    pose2.toCart(x,y);

    // x = -map.height * map.resolution / 2 + map.resolution / 2.0 + row * map.resolution;

    size_t row = size_t (abs(x/map.resolution + map.height * 0.5 - 0.5));
    size_t col = size_t (abs(y/map.resolution + map.width * 0.5 - 0.5));

    if(row >= map.height || col >= map.width) return index;
    else return row + col * map.width;
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

size_t CartesianGrid::maxProbCellIndex()
{
    double max = posterior.at(0);
    size_t _cell_index = 0;

    for(size_t i = 1; i < grid_size; i++){ _cell_index = (max < posterior.at(i)) ? i : _cell_index;}
    return _cell_index;
}


void CartesianGrid::getLocalMaximas()
{
    new_local_maxima.clear();
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

                new_local_maxima.push_back(tmp_new);
                filter_grid.at(k) = 1;
                for(size_t n = 0; n < neighbors.size(); n++){
                    filter_grid.at(neighbors.at(n)) = 0;
                }
            }
        }
        neighbors.clear();
    }
}

void CartesianGrid::trackLocalMaximas()
{
    LocalMaxima_t tmp_match;
    matched_local_maxima = old_local_maxima;
    main_local_maxima.clear();
    int8_t counter_threshold = 5;

    if(old_local_maxima.empty()){
        old_local_maxima = new_local_maxima;
        return;
    }

    std::vector<bool> find_match(new_local_maxima.size(),false);
    double dist_threshold = 2 * map.resolution;

    for(size_t i = 0; i < new_local_maxima.size(); i++){
        for(size_t j = 0; j < old_local_maxima.size(); j++){

            if(cellsDistance(new_local_maxima.at(i).index, old_local_maxima.at(j).index) < dist_threshold && !find_match.at(i)){
                matched_local_maxima.at(j).index = new_local_maxima.at(i).index;
                matched_local_maxima.at(j).counter = old_local_maxima.at(j).counter + 1;
                find_match.at(i) = true;
            }
        }

        if(!find_match.at(i)){
            tmp_match.index = new_local_maxima.at(i).index;
            tmp_match.counter = 1;
            tmp_match.tracking = false;
            matched_local_maxima.push_back(tmp_match);
        }
    }
    ROS_ASSERT(old_local_maxima.size() <= matched_local_maxima.size());

    for(size_t k = 0; k < old_local_maxima.size(); k++){
        if(matched_local_maxima.at(k).counter == old_local_maxima.at(k).counter){
            matched_local_maxima.at(k).counter = old_local_maxima.at(k).counter - 1;
        }
    }

    old_local_maxima.clear();

    for(size_t a = 0; a < matched_local_maxima.size(); a++){
        size_t index = matched_local_maxima.at(a).index;

        if(posterior.at(index) < posterior.at(0)) matched_local_maxima.at(a).counter = -counter_threshold-1;

        if(matched_local_maxima.at(a).counter > counter_threshold){
            matched_local_maxima.at(a).counter = counter_threshold +1;
            if(!matched_local_maxima.at(a).tracking) matched_local_maxima.at(a).tracking = true;
        }
        if(matched_local_maxima.at(a).counter <= 0){
            matched_local_maxima.at(a).tracking = false;
        } else {
            old_local_maxima.push_back(matched_local_maxima.at(a));
        }

        if(matched_local_maxima.at(a).tracking == true )
            main_local_maxima.push_back(matched_local_maxima.at(a));
    }
    // Don't Track!
    main_local_maxima.clear();
    main_local_maxima = new_local_maxima;

    local_maxima_poses.poses.clear();
    geometry_msgs::Pose tmp_pose;
    for(size_t j = 0; j < main_local_maxima.size(); j++){
        uint index = main_local_maxima.at(j).index;
        tmp_pose.position.x = map.cell.at(index).cartesian.x;
        tmp_pose.position.y = map.cell.at(index).cartesian.y;
        tmp_pose.position.z = 0.0;
        local_maxima_poses.poses.push_back(tmp_pose);
    }
    local_maxima_poses.header.frame_id = "base_footprint";
    local_maxima_poses.header.stamp = ros::Time::now();
}

void CartesianGrid::trackMaxProbability()
{
    uint8_t loop_rate = 5;
    int8_t counter_threshold = 5 * loop_rate;
//    double dist_threshold = 4 * map.resolution;
    double dist_threshold = 1.00;
    double probability_threshold = 2 / 3 * (cell_probability.unknown - cell_probability.free);

    if(main_local_maxima.empty() && last_highest_lm.index == 0){

        highest_prob_point.point.x = map.cell.at(last_highest_lm.index).cartesian.x;
        highest_prob_point.point.y = map.cell.at(last_highest_lm.index).cartesian.y;
        highest_prob_point.point.z = 0.0;
        highest_prob_point.header.frame_id = "base_footprint";
        highest_prob_point.header.stamp = ros::Time::now();
        return;
    }

    size_t max_index = 0;

    for(size_t i = 0; i < main_local_maxima.size(); i++){
        size_t index = main_local_maxima.at(i).index;

        if(posterior.at(index) > posterior.at(max_index)){
            max_index = index;
        }
    }

    if(!main_local_maxima.empty() && last_highest_lm.index == 0){
        last_highest_lm.index = max_index;
    }
    double range1 = map.cell.at(max_index).polar.range;
    double range2 = map.cell.at(last_highest_lm.index).polar.range;
    double min_range = (range1 + range2) / 2.0;

    Velocity_t robot_max_velocity;
    robot_max_velocity.linear = 1.0;
    robot_max_velocity.angular = 1.0;

    double max_angular_distance = robot_max_velocity.angular * diff_time.toSec() * min_range;
    double max_linear_distance = robot_max_velocity.linear * diff_time.toSec();
    double tracking_distance = maximum(dist_threshold, max_linear_distance, max_angular_distance);
    ROS_INFO("tracking_distance:    %.2f", tracking_distance);
    ROS_INFO("max_angular_distance:    %.2f", max_angular_distance);
    ROS_INFO("max_linear_distance:    %.2f", max_linear_distance);

    if(cellsDistance(max_index, last_highest_lm.index) < tracking_distance){
        last_highest_lm.index = max_index;
        last_highest_lm.counter++;
    }
    else {
        if((posterior.at(max_index) - posterior.at(last_highest_lm.index)) < probability_threshold){
            last_highest_lm.counter++;
        } else
            last_highest_lm.counter--;
    }

    if(last_highest_lm.counter > counter_threshold){
        last_highest_lm.tracking = true;
        last_highest_lm.counter = counter_threshold + 1;
        ROS_INFO("1- keep tracking  %d", last_highest_lm.counter);
        goto stop;
    } else if(last_highest_lm.counter < 0){
        last_highest_lm.index = max_index;
        last_highest_lm.counter = counter_threshold + 1;
        ROS_ERROR("2- change highest point  %d", last_highest_lm.counter);

        goto stop;
    } else {
        size_t predicted_highest_lm = predictHighestProbability(last_highest_lm.index);
//        size_t temp_lm = max_index;
        size_t temp_lm = last_highest_lm.index;
        double min_distance = 2 * map.resolution;

        for(size_t i = 0; i < main_local_maxima.size(); i++){
            size_t in = main_local_maxima.at(i).index;
            double temp_dist = cellsDistance(in, predicted_highest_lm);
//            temp_lm = (temp_dist < min_distance) ? in : temp_lm ;
            temp_lm = (temp_dist < tracking_distance) ? in : temp_lm ;

        }
        last_highest_lm.index = (fabs(velocity.linear) > 1e-4 || fabs(velocity.angular) > 1e-4)
                ? temp_lm : last_highest_lm.index;
        ROS_INFO("3- in between  %d", last_highest_lm.counter);
        goto stop;
    }

//    //DO NOT TRACK HIGHEST PROBABILITY POINT
//    last_highest_lm.index = max_index;
//    last_highest_lm.tracking = true;
    stop:
//    ROS_INFO("diff_time:    %.4f    loop rate:  %d", diff_time.toSec(), int(1/diff_time.toSec()));

    if(last_highest_lm.tracking){
        highest_prob_point.point.x = map.cell.at(last_highest_lm.index).cartesian.x;
        highest_prob_point.point.y = map.cell.at(last_highest_lm.index).cartesian.y;
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
    double diff_x = map.cell.at(c1).cartesian.x - map.cell.at(c2).cartesian.x;
    double diff_y = map.cell.at(c1).cartesian.y - map.cell.at(c2).cartesian.y;
    return (sqrt(diff_x*diff_x + diff_y*diff_y));
}


CartesianGrid::~CartesianGrid()
{}







