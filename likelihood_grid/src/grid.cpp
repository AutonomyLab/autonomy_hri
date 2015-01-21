#include "grid.h"
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

Grid::Grid(uint32_t map_size,
                             SensorFOV_t _sensor_fov,
                             double_t map_resolution,
                             CellProbability_t _cell_probability,
                             double _target_detection_probability,
                             double _false_positive_probability)
{
    ROS_INFO("Constructing an instace of cartesian likelihood grid.");
    ROS_ASSERT(map_size % 2 == 0);
    lk_ = ros::Time::now();
    map.height = map_size; // DEFAULT 80
    map.width = map_size; // DEFAULT 80
    map.resolution = map_resolution; // DEFAULT 0.25
    map.origin.position.x = (double) map.height*map.resolution / -2.0;
    map.origin.position.y = (double) map.width*map.resolution / -2.0;

    cell_probability = _cell_probability;
    TARGET_DETECTION_PROBABILITY_ = _target_detection_probability;
    FALSE_DETECTION_PROBABILITY_ = _false_positive_probability;

    grid_size = map.height * map.width; // DEFAULT 6400
    sensor_fov = _sensor_fov;

    x_.max = map.height * map.resolution / 2;
    y_.max = map.width * map.resolution / 2;
    x_.min = -x_.max;
    y_.min = -y_.max;

    ROS_INFO("Map Info: Height: %d      Width:  %d      Resolution: %.2f    origin: (%.2f, %.2f)",
             map.height,
             map.width,
             map.resolution,
             map.origin.position.x,
             map.origin.position.y);

    ROS_INFO("Grid Info: size: %d      x_max:  %.2f      y_max: %.2f",
             grid_size,
             x_.max,
             y_.max);

    last_highest_lm_.index = 0;
    last_highest_lm_.tracking = false;
    last_highest_lm_.counter = 0;

//    8 5 2
//    7 4 1
//    6 3 0

    Cell_t temp_cell;
    size_t i = 0;

    for(size_t c = 0; c < map.width; c++){
        for(size_t r = 0; r < map.height; r++){

            temp_cell.cartesian.x = x_.min + map.resolution / 2.0 + r * map.resolution;
            temp_cell.cartesian.y = y_.min + map.resolution / 2.0 + c * map.resolution;
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
    ROS_ASSERT( temp_cell.cartesian.x > -x_.max );
    ROS_ASSERT( temp_cell.cartesian.y > -y_.max );

    posterior.resize(grid_size, cell_probability.unknown);
    prior.resize(grid_size, cell_probability.unknown);
    true_likelihood_.resize(grid_size, cell_probability.unknown);
    false_likelihood_.resize(grid_size, cell_probability.unknown);

    predicted_posterior_.resize(grid_size, cell_probability.unknown);
    predicted_true_likelihood_.resize(grid_size, cell_probability.unknown);
    predicted_false_likelihood_.resize(grid_size, cell_probability.unknown);

    setOutFOVProbability(posterior, cell_probability.unknown);
    setOutFOVProbability(true_likelihood_, cell_probability.unknown);
    setOutFOVProbability(false_likelihood_, cell_probability.unknown);

    setOutFOVProbability(prior, cell_probability.unknown);
    setOutFOVProbability(predicted_posterior_, cell_probability.unknown);
    setOutFOVProbability(predicted_true_likelihood_, cell_probability.unknown);
    setOutFOVProbability(predicted_false_likelihood_, cell_probability.unknown);
}

bool Grid::sortByProbability(LocalMaxima_t &i, LocalMaxima_t &j){
return (posterior[i.index] < posterior[j.index]);
}

void Grid::setOutFOVProbability(std::vector<double> &data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(!map.cell_inFOV.at(i)) { data[i] = val; }
    }
}

void Grid::setInFOVProbability(std::vector<double>& data, const double val){
    for(size_t i = 0; i < grid_size; i++){
        if(map.cell_inFOV.at(i)) { data[i] = val; }
    }
}

void Grid::computeLikelihood(const std::vector<PolarPose>& pose,
                            std::vector<double> &_true_likelihood,
                            std::vector<double> &_false_likelihood)
{
//    double range_guassian_factor = cell_probability.human / normalDistribution(0.0, 0.0, stdev.range);
//    double angle_guassian_factor = cell_probability.human / normalDistribution(0.0, 0.0, toRadian(stdev.angle));

    for(size_t i = 0; i < grid_size; i++){
        double range = map.cell.at(i).polar.range;
        double angle = map.cell.at(i).polar.angle;
        double detection_likelihood = cell_probability.unknown;
        double miss_detection_likelihood = cell_probability.unknown;

        double cell_prob = 0.0;
        if(map.cell_inFOV.at(i)){
            if(pose.empty()) miss_detection_likelihood = cell_probability.human; // ?????

            else{
                for(size_t p = 0; p < pose.size(); p++){
                    double Gr = (pose.at(p).range < 0.01) ? 1.0 : normalDistribution(range, pose.at(p).range, stdev.range);
                    double Ga = normalDistribution(angle, pose.at(p).angle, toRadian(stdev.angle));
                    cell_prob += Gr * Ga;
                }
                detection_likelihood = (((cell_prob)/(pose.size()) < cell_probability.free) ? cell_probability.free : (cell_prob)/(pose.size()));
            }
        }

        _true_likelihood[i] = (detection_likelihood * TARGET_DETECTION_PROBABILITY_) + (miss_detection_likelihood * (1.0 - TARGET_DETECTION_PROBABILITY_));
        _false_likelihood[i] = (detection_likelihood * FALSE_DETECTION_PROBABILITY_) + (miss_detection_likelihood * (1.0 - FALSE_DETECTION_PROBABILITY_));
    }
}


void Grid::updateGridProbability(std::vector<double>& _prior,
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
        if((!map.cell_inFOV.at(i)) && (_posterior[i] < cell_probability.unknown ))
        {_posterior[i] = cell_probability.unknown;}
    }
}

void Grid::bayesOccupancyFilter()
{
    // PREDICTION BY MOTION MODEL
    computeLikelihood(polar_array.predicted, predicted_true_likelihood_, predicted_false_likelihood_);
    updateGridProbability(prior, predicted_true_likelihood_, predicted_false_likelihood_, predicted_posterior_);

    // UPDATE BY OBSERVATION
    computeLikelihood(polar_array.current, true_likelihood_, false_likelihood_);
    updateGridProbability(predicted_posterior_, true_likelihood_, false_likelihood_, posterior);

    prior = posterior;
    polar_array.past = polar_array.current;
    max_probability_ = posterior.at(maxProbCellIndex());
}


void Grid::fuse(const std::vector<double> &data_1,
                         const std::vector<double> &data_2,
                         const std::vector<double> &data_3,
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
        for(size_t i = 0; i < grid_size; i++)
            {posterior.at(i) = (data_1.at(i) + data_2.at(i) + data_3.at(i))/3.0;}
    }
}


void Grid::getPose(geometry_msgs::PoseArray& crtsn_array)
{
    polar_array.current.clear();

    PolarPose polar_pose_point;

    for(size_t i = 0; i < crtsn_array.poses.size(); i++){

        polar_pose_point.fromCart(crtsn_array.poses.at(i).position.x,
                                  crtsn_array.poses.at(i).position.y);

        polar_array.current.push_back(polar_pose_point);

    }
}

void Grid::getPose(const autonomy_human::raw_detectionsConstPtr torso_img)
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
//        torso_polar_pose.range = 0.0000952381 * h * h - 0.0696716 * h + 14.4483;
        polar_array.current.push_back(torso_polar_pose);
    }
}

void Grid::getPose(const hark_msgs::HarkSourceConstPtr& sound_src)
{
    ros::Duration d = ros::Time::now() - lk_;
    lk_ = ros::Time::now();
    double angle2;
    if(d.toSec() > 1.0 / 10)
        polar_array.current.clear();
    PolarPose sound_src_polar;

    for(size_t i = 0; i < sound_src->src.size(); i++){
        sound_src_polar.range = -1.0;
        sound_src_polar.angle = 2 * M_PI;

        if(fabs(sound_src->src.at(i).y) > 1e-9 && sound_src->src.at(i).power > 28.0){
//            sound_src_polar.range = sqrt(pow(sound_src->src[i].x*2,2) + pow(sound_src->src[i].y*2,2));
//            sound_src_polar.range = sensor_fov.range.max/2.0;

            sound_src_polar.angle = toRadian(sound_src->src.at(i).azimuth);
            polar_array.current.push_back(sound_src_polar);
            if(sound_src->src.at(i).azimuth >= 0.0)  angle2 = 180.0 - sound_src->src.at(i).azimuth;
            else angle2 = -180.0 - sound_src->src.at(i).azimuth;
            sound_src_polar.angle = toRadian(angle2);
            polar_array.current.push_back(sound_src_polar);
        }
    }
}

void Grid::predict(const Velocity_t _robot_velocity)
{
    polar_array.predicted.clear();
    polar_array.predicted = polar_array.past;

    geometry_msgs::Point pose1, pose2;
    PolarPose polar1, polar2;

    velocity_.linear = -_robot_velocity.linear;
    velocity_.angular = -_robot_velocity.angular;
    velocity_.lin.x = -_robot_velocity.lin.x;
    velocity_.lin.y = -_robot_velocity.lin.y;


    for(size_t p = 0 ; p < polar_array.past.size(); p++){
        polar1 = polar_array.past.at(p);
        polar1.toCart(pose1.x, pose1.y);

        pose2.x = velocity_.lin.x * diff_time.toSec() + pose1.x;
        pose2.y = velocity_.lin.y * diff_time.toSec() + pose1.y;

        polar2.fromCart(pose2.x, pose2.y);
        polar_array.predicted.at(p) = polar2;

//        polar_array.predicted.at(p).range = velocity.linear * diff_time.toSec() + polar_array.past.at(p).range;
        polar_array.predicted.at(p).angle = velocity_.angular * diff_time.toSec() + polar_array.past.at(p).angle;
    }
}

size_t Grid::predictHighestProbability(size_t index)
{

    geometry_msgs::Point pose1, pose2;
    PolarPose polar1, polar2;
    double x, y;


    polar1.range = map.cell.at(index).polar.range;
    polar1.angle = map.cell.at(index).polar.angle;
    polar1.angle = velocity_.angular * diff_time.toSec() + polar1.angle;
    polar1.toCart(pose1.x, pose1.y);

    x = velocity_.lin.x * diff_time.toSec() + pose1.x;
    y = velocity_.lin.y * diff_time.toSec() + pose1.y;

//    polar2.fromCart(pose2.x, pose2.y);
//    polar_array.predicted.at(p) = polar2;



//    PolarPose pose1, pose2;

//    pose1.range = map.cell.at(index).polar.range;
//    pose1.angle = map.cell.at(index).polar.angle;

//    pose2.range = velocity.linear * diff_time.toSec() + pose1.range;
//    pose2.angle = velocity.angular * diff_time.toSec() + pose1.angle;
//    pose2.toCart(x,y);

    // x = -map.height * map.resolution / 2 + map.resolution / 2.0 + row * map.resolution;

    size_t row = size_t (abs(x/map.resolution + map.height * 0.5 - 0.5));
    size_t col = size_t (abs(y/map.resolution + map.width * 0.5 - 0.5));

    if(row >= map.height || col >= map.width) return index;
    else return row + col * map.width;
}

void Grid::polar2Crtsn(std::vector<PolarPose>& polar_array,
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

size_t Grid::maxProbCellIndex()
{
    double max = posterior.at(0);
    size_t _cell_index = 0;

    for(size_t i = 1; i < grid_size; i++){ _cell_index = (max < posterior.at(i)) ? i : _cell_index;}
    return _cell_index;
}


void Grid::getLocalMaximas()
{
    new_local_maxima_.clear();
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
        tmp_new.probability = posterior.at(k);
        lm = false;
        uint8_t col = k / map.height;
        uint8_t row = k % map.height;

        if ((tmp_new.probability <= posterior.at(0)) ||(filter_grid.at(k) == 0)) filter_grid.at(k) = 0;

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

                    if(tmp_new.probability >= posterior.at(x)){ // if the cell(k) has higher prob. than its neighbor(x)
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

                if(tmp_new.probability > cell_probability.unknown)
                    new_local_maxima_.push_back(tmp_new);
                filter_grid.at(k) = 1;
                for(size_t n = 0; n < neighbors.size(); n++){
                    filter_grid.at(neighbors.at(n)) = 0;
                }
            }
        }
        neighbors.clear();
    }
}

void Grid::trackLocalMaximas()
{
    LocalMaxima_t tmp_match;
    matched_local_maxima_ = old_local_maxima_;
    main_local_maxima_.clear();
    int8_t counter_threshold = 5;

    if(old_local_maxima_.empty()){
        old_local_maxima_ = new_local_maxima_;
        return;
    }

    std::vector<bool> find_match(new_local_maxima_.size(),false);
    double dist_threshold = 2 * map.resolution;

    for(size_t i = 0; i < new_local_maxima_.size(); i++){
        for(size_t j = 0; j < old_local_maxima_.size(); j++){

            if(cellsDistance(new_local_maxima_.at(i).index, old_local_maxima_.at(j).index) < dist_threshold && !find_match.at(i)){
                matched_local_maxima_.at(j).index = new_local_maxima_.at(i).index;
                matched_local_maxima_.at(j).counter = old_local_maxima_.at(j).counter + 1;
                find_match.at(i) = true;
            }
        }

        if(!find_match.at(i)){
            tmp_match.index = new_local_maxima_.at(i).index;
            tmp_match.counter = 1;
            tmp_match.tracking = false;
            matched_local_maxima_.push_back(tmp_match);
        }
    }
    ROS_ASSERT(old_local_maxima_.size() <= matched_local_maxima_.size());

    for(size_t k = 0; k < old_local_maxima_.size(); k++){
        if(matched_local_maxima_.at(k).counter == old_local_maxima_.at(k).counter){
            matched_local_maxima_.at(k).counter = old_local_maxima_.at(k).counter - 1;
        }
    }

    old_local_maxima_.clear();

    for(size_t a = 0; a < matched_local_maxima_.size(); a++){
//        size_t index = matched_local_maxima.at(a).index;

        if(matched_local_maxima_.at(a).probability < posterior.at(0)) matched_local_maxima_.at(a).counter = -counter_threshold-1;

        if(matched_local_maxima_.at(a).counter > counter_threshold){
            matched_local_maxima_.at(a).counter = counter_threshold +1;
            if(!matched_local_maxima_.at(a).tracking) matched_local_maxima_.at(a).tracking = true;
        }
        if(matched_local_maxima_.at(a).counter <= 0){
            matched_local_maxima_.at(a).tracking = false;
        } else {
            old_local_maxima_.push_back(matched_local_maxima_.at(a));
        }

        if(matched_local_maxima_.at(a).tracking == true )
            main_local_maxima_.push_back(matched_local_maxima_.at(a));
    }
    // Don't Track!
    main_local_maxima_.clear();
    main_local_maxima_ = new_local_maxima_;
    std::sort (main_local_maxima_.begin(), main_local_maxima_.end());

    local_maxima_poses.poses.clear();
    geometry_msgs::Pose tmp_pose;
    for(size_t j = 0; j < main_local_maxima_.size(); j++){
        uint index = main_local_maxima_.at(j).index;
        tmp_pose.position.x = map.cell.at(index).cartesian.x;
        tmp_pose.position.y = map.cell.at(index).cartesian.y;
        tmp_pose.position.z = 0.0;
        local_maxima_poses.poses.push_back(tmp_pose);
//        ROS_INFO("lm %lu    x:%.2f  y:%.2f   p:%.4f", j, tmp_pose.position.x, tmp_pose.position.y, main_local_maxima.at(j).probability);
    }
    local_maxima_poses.header.frame_id = "base_footprint";
    local_maxima_poses.header.stamp = ros::Time::now();
}

void Grid::trackMaxProbability()
{
    uint8_t loop_rate = 10;
    int8_t counter_threshold = 1 * loop_rate;
    double dist_threshold = 1.00; //4 * map.resolution;
    double probability_threshold = (cell_probability.unknown - cell_probability.free) / 3.0;


    //FIND THE CELL WITH MAXIMUM PROBABILITY

    size_t max_index = 0;

    if(!main_local_maxima_.empty()){
        std::sort (main_local_maxima_.begin(), main_local_maxima_.end());
        max_index = main_local_maxima_.at(main_local_maxima_.size()-1).index;
    }
//    for(size_t i = 0; i < main_local_maxima.size(); i++){
//        size_t index = main_local_maxima.at(i).index;
//        max_index = (posterior.at(index) > posterior.at(max_index)) ? index : max_index;
//    }

    if(last_highest_lm_.index == 0)
        last_highest_lm_.index = (main_local_maxima_.empty()) ? last_highest_lm_.index : max_index;

    Velocity_t robot_max_velocity;
    robot_max_velocity.linear = 1.0; //TODO: MAKE THESE PARAMETERS
    robot_max_velocity.angular = 1.0;
    double range1 = map.cell.at(max_index).polar.range;
    double range2 = map.cell.at(last_highest_lm_.index).polar.range;
    double min_range = (range1 + range2) / 2.0;
    double max_angular_distance = robot_max_velocity.angular * diff_time.toSec() * min_range;
    double max_linear_distance = robot_max_velocity.linear * diff_time.toSec();
    double tracking_distance = maximum(dist_threshold, max_linear_distance, max_angular_distance);

    if(cellsDistance(max_index, last_highest_lm_.index) < tracking_distance){
        last_highest_lm_.index = max_index;
        last_highest_lm_.counter++;
    }
    else {
        if((posterior.at(max_index) - posterior.at(last_highest_lm_.index)) < probability_threshold){
            last_highest_lm_.counter++;
        } else last_highest_lm_.counter--;
    }

    if(last_highest_lm_.counter > counter_threshold){
        last_highest_lm_.tracking = true;
        last_highest_lm_.counter = counter_threshold + 1;
        goto stop;
    } else if(last_highest_lm_.counter < 0){
        last_highest_lm_.index = max_index;
        last_highest_lm_.counter = counter_threshold + 1;

        goto stop;
    } else {
        size_t predicted_highest_lm = predictHighestProbability(last_highest_lm_.index);
        size_t temp_lm = last_highest_lm_.index;

        for(size_t i = 0; i < main_local_maxima_.size(); i++){

            size_t lm_index = main_local_maxima_.at(i).index;
            double temp_dist = cellsDistance(lm_index, predicted_highest_lm);
            temp_lm = (temp_dist < tracking_distance) ? lm_index : temp_lm ;
        }

        last_highest_lm_.index = (fabs(velocity_.linear) > 1e-4 || fabs(velocity_.angular) > 1e-4)
                ? temp_lm : last_highest_lm_.index;
        goto stop;
    }

//    //DO NOT TRACK HIGHEST PROBABILITY POINT
//    last_highest_lm.index = max_index;
//    last_highest_lm.tracking = true;

    stop:

    highest_prob_point.point.x = map.cell.at(last_highest_lm_.index).cartesian.x;
    highest_prob_point.point.y = map.cell.at(last_highest_lm_.index).cartesian.y;
    highest_prob_point.point.z = 0.0;
    highest_prob_point.header.frame_id = "base_footprint";
    highest_prob_point.header.stamp = ros::Time::now();
}

void Grid::updateLocalMaximas()
{
    getLocalMaximas();
    trackLocalMaximas();
    trackMaxProbability();
}

double Grid::cellsDistance(size_t c1, size_t c2)
{
    double diff_x = map.cell.at(c1).cartesian.x - map.cell.at(c2).cartesian.x;
    double diff_y = map.cell.at(c1).cartesian.y - map.cell.at(c2).cartesian.y;
    return (sqrt(diff_x*diff_x + diff_y*diff_y));
}


Grid::~Grid()
{}







