#include <ros/ros.h>
#include "grid_interface.h"

#define _USE_MATH_DEFINES
#define _LOOPRATE 4

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    ros::Rate looprate(_LOOPRATE);
    tf::TransformListener *tf_listener;


    GridFOV_t _global_fov;

    ros::param::param("~/LikelihoodGrid/grid_angle_min",_global_fov.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",_global_fov.angle.max, M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_resolution",_global_fov.angle.resolution, M_PI/18);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf: resolution: %.2lf",
             _global_fov.angle.min,
             _global_fov.angle.max,
             _global_fov.angle.resolution);

    ros::param::param("~/LikelihoodGrid/grid_range_min",_global_fov.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",_global_fov.range.max, 40.0);
    ros::param::param("~/LikelihoodGrid/grid_range_resolution",_global_fov.range.resolution, 0.5);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf: resolution: %.2lf",
             _global_fov.range.min,
             _global_fov.range.max,
             _global_fov.range.resolution);

    double _update_rate;
    ros::param::param("~/LikelihoodGrid/update_rate",_update_rate, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",_update_rate);

    double _human_cell_probability = 1.0; //TODO: MAKE IT ROS PARAM
    ros::param::param("~/LikelihoodGrid/human_cell_probability",_human_cell_probability, 1.0);
    ROS_INFO("/LikelihoodGrid/human_cell_probability is set to %.2lf",_human_cell_probability);

    double _free_cell_probability;
    ros::param::param("~/LikelihoodGrid/free_cell_probability",_free_cell_probability, 0.1);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %.2lf",_free_cell_probability);

    double _unknown_cell_probability;
    ros::param::param("~/LikelihoodGrid/unknown_cell_probability",_unknown_cell_probability, 0.5);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability is set to %.2lf",_unknown_cell_probability);

    int _number_of_sensors;
    ros::param::param("~/LikelihoodGrid/number_of_sensors",_number_of_sensors, 2);
    ROS_INFO("/LikelihoodGrid/number_of_sensors is set to %u",_number_of_sensors);

    int _sensitivity;
    ros::param::param("~/LikelihoodGrid/sensitivity",_sensitivity, 1);
    ROS_INFO("/LikelihoodGrid/sensitivity is set to %u",_sensitivity);

    double _update_time_ratio;
    ros::param::param("~/update_time_ratio",_update_time_ratio, 10.0);
    ROS_INFO("/update_time_ratio is set to %.2lf",_update_time_ratio);

    //CALCULATING THE PRIOR!
    double upper_bound, lower_bound;
    upper_bound = pow(_free_cell_probability, _number_of_sensors - _sensitivity) * pow(_human_cell_probability,_sensitivity);
    lower_bound = pow(_free_cell_probability, _number_of_sensors - _sensitivity + 1) * pow(_human_cell_probability,_sensitivity - 1);
    _unknown_cell_probability = pow((upper_bound + lower_bound)/2, 1.0/_number_of_sensors);
    ROS_INFO("/LikelihoodGrid/unknown_cell_probability has been changed to %.2lf",_unknown_cell_probability);

    CellProbability_t _cell_probability;
    _cell_probability.free = _free_cell_probability;
    _cell_probability.human = _human_cell_probability;
    _cell_probability.unknown = _unknown_cell_probability;

// CREATE AN INSTANCE OF LIKELIHOOD GRID INTERFACE WITH PROPER PARAMETERS
    GridInterface likelihood_grid_interface(n,
                                  tf_listener,
                                  _global_fov,
                                  _update_rate,
                                  _update_time_ratio,
                                  _cell_probability);
    // FOR EVERY HUMAN FEATURE DATA (E.G. LEGS, FACES, SOUND)



    // DEFINE THE SENSOR FOV
    GridFOV_t laser_fov = _global_fov;
    laser_fov.range.max = 20.0;
    laser_fov.angle.min = toRadian(-120.0);//-2.35619449615;
    laser_fov.angle.max = toRadian(120.0);//2.35619449615;
    GridFOV_t camera_fov = _global_fov;
    camera_fov.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
    camera_fov.range.max = 10.00; // TODO: MAKE SURE OF THE REAL FOV
    camera_fov.angle.min = toRadian(-65.0/2); // -65/2 degree
    camera_fov.angle.max = toRadian(65.0/2); // 65/2 degree


    // UPDATE THE SPECIFIC HUMAN GRID FOV
    likelihood_grid_interface.initLegs(laser_fov);
    likelihood_grid_interface.initFaces(camera_fov);
    likelihood_grid_interface.initHuman();

    // SUBSCRIBE TO THE PROPER TOPIC
    ros::Subscriber legs_sub = n.subscribe("legs",10,
                                           &GridInterface::legCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber faces_sub = n.subscribe("human",10,
                                           &GridInterface::faceCallBack,
                                           &likelihood_grid_interface);

    while (ros::ok()) {
        //ROS_ASSERT(0);

        // IN EVERY LOOP:
            // UPDATE ALL THE AVAILABLE HUMAN FEATURE LIKELIHOOD GRIDS AND PUBLISH THEM
            // FUSE THEM AND UPDATE THE GLOBAL HUMAN LIKELIHOOD GRID AND PUBLISH IT
       // lkGridInterface.spin(looprate.cycleTime().toSec());
        likelihood_grid_interface.spin();
        ros::spinOnce();
        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");

    }
    return 0;
}
