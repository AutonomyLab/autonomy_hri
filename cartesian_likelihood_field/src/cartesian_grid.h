#ifndef CARTESIAN_GRID_H
#define CARTESIAN_GRID_H

#include <vector>
#include <cmath>
//#include <math.h>
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
#include "polarcord.h"


double normalize(const double val, const double x_min, const double x_max, const double range_min, const double range_max);

struct FOV_t{
    double min;
    double max;
};

struct SensorFOV_t
{
    FOV_t range;
    FOV_t angle;
};

struct CellProbability_t{
    double free;
    double unknown;
    double human;
};

struct LocalMaxima_t{
    size_t index;
    bool tracking;
    int8_t counter;
    double probability;
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
    double linear;
    double angular;
};

struct MapMetaData_t{
double resolution; // The map resolution [m/cell]
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


class CartesianGrid
{
private:
    FOV_t x_;
    FOV_t y_;

    ros::Time lk_;

    size_t maxProbCellIndex();
    double cellsDistance(size_t c1, size_t c2);
    std::vector<LocalMaxima_t> old_local_maxima_;
    std::vector<LocalMaxima_t> new_local_maxima_;
    std::vector<LocalMaxima_t> matched_local_maxima_;
    std::vector<LocalMaxima_t> main_local_maxima_;

    std::vector<double> true_likelihood_;
    std::vector<double> false_likelihood_;
    std::vector<double> predicted_posterior_;
    std::vector<double> predicted_true_likelihood_;
    std::vector<double> predicted_false_likelihood_;

    LocalMaxima_t last_highest_lm_;
    Velocity_t velocity_;
    Velocity_t last_velocity_;

    double TARGET_DETECTION_PROBABILITY_;
    double FALSE_DETECTION_PROBABILITY_;
    double max_probability_;

    void computeLikelihood(const std::vector<PolarPose>& pose,
                                std::vector<double> &_true_likelihood,
                                std::vector<double> &_false_likelihood);
    void updateGridProbability(std::vector<double> &_prior,
                               const std::vector<double> &_true_likelihood,
                               const std::vector<double> &_false_likelihood,
                               std::vector<double> &_posterior);
    void setOutFOVProbability(std::vector<double>& data, const double val);
    void setInFOVProbability(std::vector<double>& data, const double val);
    void getLocalMaximas();
    void trackLocalMaximas();
    bool sortByProbability(LocalMaxima_t &i, LocalMaxima_t &j);

public:
    MapMetaData_t map;
    uint32_t grid_size;
    PolarPose stdev;
    CellProbability_t cell_probability;
    SensorFOV_t sensor_fov;

    std::vector<double> posterior;
    std::vector<double> prior;

    ros::Time last_time;
    ros::Duration diff_time;

    Cycle_t<geometry_msgs::PoseArray> crtsn_array;
    Cycle_t<std::vector<PolarPose> > polar_array;

    geometry_msgs::PoseArray local_maxima_poses;
    geometry_msgs::PointStamped highest_prob_point;


    CartesianGrid(uint32_t map_size,
                  SensorFOV_t _sensor_fov,
                  double_t map_resolution,
                  CellProbability_t _cell_probability,
                  double _target_detection_probability,
                  double _false_positive_probability);
    CartesianGrid();
    ~CartesianGrid();

    void fuse(const std::vector<double> data_1, const std::vector<double> data_2,
              const std::vector<double> data_3, bool multiply);
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
};

#endif
