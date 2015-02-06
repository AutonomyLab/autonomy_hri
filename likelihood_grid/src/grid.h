#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include <boost/math/distributions/normal.hpp>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <hark_msgs/HarkSource.h>
#include <hark_msgs/HarkSourceVal.h>
#include <autonomy_human/human.h>
#include <autonomy_human/raw_detections.h>
#include <nav_msgs/OccupancyGrid.h>
#include "polarcord.h"

float normalize(const float val, const float x_min, const float x_max, const float range_min, const float range_max);

float pointDistance(geometry_msgs::Point a, geometry_msgs::Point b);

struct FOV_t{
    float min;
    float max;
};

struct SensorFOV_t
{
    FOV_t range;
    FOV_t angle;
};

struct CellProbability_t{
    float free;
    float unknown;
    float human;
};

struct LocalMaxima_t{
    size_t index;
    bool tracking;
    int8_t counter;
    float probability;
    bool operator < (const LocalMaxima_t& lm) const
    {
        return (probability < lm.probability);
    }
};

struct Cell_t{
    geometry_msgs::Point32    cartesian;
    PolarPose               polar;
};

struct Velocity_t{
    geometry_msgs::Point lin;
    float linear;
    float angular;
};

struct MapMetaData_t{
float resolution; // The map resolution [m/cell]
uint32_t width; //Map width [cells]
uint32_t height; // Map height [cells]
geometry_msgs::Pose origin; // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
std::vector<Cell_t> cell; // robot-centric position of the cell (r,c) in the map
std::vector<bool> cell_inFOV; // is cell(r,c) in sensor fov?
};

template <typename T>
struct Cycle_t{
    T current;
    T past;
    T predicted;
};


class CGrid
{
private:
    FOV_t x_;
    FOV_t y_;

    ros::Time lk_;

    size_t maxProbCellIndex();
    float cellsDistance(size_t c1, size_t c2);
    std::vector<LocalMaxima_t> old_local_maxima_;
    std::vector<LocalMaxima_t> new_local_maxima_;
    std::vector<LocalMaxima_t> matched_local_maxima_;
    std::vector<LocalMaxima_t> main_local_maxima_;

    std::vector<float> true_likelihood_;
    std::vector<float> false_likelihood_;
    std::vector<float> predicted_posterior_;
    std::vector<float> predicted_true_likelihood_;
    std::vector<float> predicted_false_likelihood_;

    LocalMaxima_t last_highest_lm_;
    Velocity_t velocity_;
    Velocity_t last_velocity_;

    float TARGET_DETECTION_PROBABILITY_;
    float FALSE_DETECTION_PROBABILITY_;
    float max_probability_;

    void computeLikelihood(const std::vector<PolarPose>& pose,
                                std::vector<float> &_true_likelihood,
                                std::vector<float> &_false_likelihood);
    void updateGridProbability(std::vector<float> &_prior,
                               const std::vector<float> &_true_likelihood,
                               const std::vector<float> &_false_likelihood,
                               std::vector<float> &_posterior);
    void setOutFOVProbability(std::vector<float>& data, const float val);
    void setInFOVProbability(std::vector<float>& data, const float val);
    void getLocalMaximas();
    void trackLocalMaximas();
    bool sortByProbability(LocalMaxima_t &i, LocalMaxima_t &j);
    float pmfr(float u, float s, float x, float d);
    float pmfa(float u, float s, float x, float d);


public:
    MapMetaData_t map;
    uint32_t grid_size;
    PolarPose stdev;
    CellProbability_t cell_probability;
    SensorFOV_t sensor_fov;
    nav_msgs::OccupancyGrid occupancy_grid;

    std::vector<float> posterior;
    std::vector<float> prior;

    ros::Time last_time;
    ros::Duration diff_time;

    Cycle_t<geometry_msgs::PoseArray> crtsn_array;
    Cycle_t<std::vector<PolarPose> > polar_array;
    Cycle_t<std::vector<PolarPose> > cov_array;

    geometry_msgs::PoseArray local_maxima_poses;
    geometry_msgs::PointStamped highest_prob_point;

    CGrid(uint32_t map_size,
                  SensorFOV_t _sensor_fov,
                  float_t map_resolution,
                  CellProbability_t _cell_probability,
                  float _target_detection_probability,
                  float _false_positive_probability);
    CGrid();
    ~CGrid();

    void fuse(const std::vector<float> &data_1, const std::vector<float> &data_2,
              const std::vector<float> &data_3, bool multiply);
    void bayesOccupancyFilter();
    void getPose(geometry_msgs::PoseArray &crtsn_array);
    void getPose(const autonomy_human::raw_detectionsConstPtr torso_img);
    void getPose(const hark_msgs::HarkSourceConstPtr& sound_src);
    void predict(const Velocity_t _robot_velocity);
    size_t predictHighestProbability(size_t index);
    void polar2Crtsn(std::vector<PolarPose> &polar_array,
                     geometry_msgs::PoseArray &crtsn_array);
    void updateLocalMaximas();
    void trackMaxProbability();
    void updateGrid(int score);

};

#endif
