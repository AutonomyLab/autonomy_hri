#include <ros/ros.h>
#include "lkgrid_interface.h"

#define _USE_MATH_DEFINES
#define _LOOPRATE 5

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    ros::Rate looprate(_LOOPRATE);
    tf::TransformListener *tf_listener;


    GridFOV_t _globalGridFOV;

    ros::param::param("~/LikelihoodGrid/grid_angle_min",_globalGridFOV.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",_globalGridFOV.angle.max, M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_resolution",_globalGridFOV.angle.resolution, M_PI/18);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf: resolution: %.2lf",
             _globalGridFOV.angle.min,
             _globalGridFOV.angle.max,
             _globalGridFOV.angle.resolution);

    ros::param::param("~/LikelihoodGrid/grid_range_min",_globalGridFOV.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",_globalGridFOV.range.max, 40.0);
    ros::param::param("~/LikelihoodGrid/grid_range_resolution",_globalGridFOV.range.resolution, 0.5);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf: resolution: %.2lf",
             _globalGridFOV.range.min,
             _globalGridFOV.range.max,
             _globalGridFOV.range.resolution);

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
    LikelihoodGridInterface lkGridInterface(n,
                                            tf_listener,
                                            _globalGridFOV,
                                            _update_rate,
                                            _update_time_ratio,
                                            _cell_probability);
    // FOR EVERY HUMAN FEATURE DATA (E.G. LEGS, FACES, SOUND)



    // DEFINE THE SENSOR FOV
    GridFOV_t legGridFOV = _globalGridFOV;
    legGridFOV.range.max = 20.0;
    legGridFOV.angle.min = toRadian(-270.0/2);
    legGridFOV.angle.max = toRadian(270.0/2);
    GridFOV_t faceGridFOV = _globalGridFOV;
    faceGridFOV.range.min = 1.00; // TODO: MAKE SURE OF THE REAL FOV
    faceGridFOV.range.max = 10.00; // TODO: MAKE SURE OF THE REAL FOV
    faceGridFOV.angle.min = toRadian(-65.0/2); // -65/2 degree
    faceGridFOV.angle.max = toRadian(65.0/2); // 65/2 degree


    // UPDATE THE SPECIFIC HUMAN GRID FOV
    lkGridInterface.init_legs(legGridFOV);
    lkGridInterface.init_faces(faceGridFOV);
    lkGridInterface.init_human();

    // SUBSCRIBE TO THE PROPER TOPIC
    ros::Subscriber legs_sub = n.subscribe("legs",10,
                                           &LikelihoodGridInterface::legs_cb,
                                           &lkGridInterface);

    ros::Subscriber faces_sub = n.subscribe("human",10,
                                           &LikelihoodGridInterface::faces_cb,
                                           &lkGridInterface);

    while (ros::ok()) {


        // IN EVERY LOOP:
            // UPDATE ALL THE AVAILABLE HUMAN FEATURE LIKELIHOOD GRIDS AND PUBLISH THEM
            // FUSE THEM AND UPDATE THE GLOBAL HUMAN LIKELIHOOD GRID AND PUBLISH IT
       // lkGridInterface.spin(looprate.cycleTime().toSec());
        lkGridInterface.spin();
        ros::spinOnce();
        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");

    }
    return 0;
}
