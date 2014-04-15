#include <ros/ros.h>
#include "lkgrid_interface.h"

#define _USE_MATH_DEFINES
#define _LOOPRATE 5


int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    ros::Rate looprate(_LOOPRATE);

    GridFOV_t _globalGridFOV;

    ros::param::param("~/LikelihoodGrid/grid_angle_min",_globalGridFOV.angle.min, -M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_max",_globalGridFOV.angle.max, M_PI);
    ros::param::param("~/LikelihoodGrid/grid_angle_resolution",_globalGridFOV.angle.resolution, M_PI/18);
    ROS_INFO("/LikelihoodGrid/grid_angle min: %.2lf max: %.2lf: resolution: %.2lf",
             _globalGridFOV.angle.min,
             _globalGridFOV.angle.max,
             _globalGridFOV.angle.resolution);

    ros::param::param("~/LikelihoodGrid/grid_range_min",_globalGridFOV.range.min, 0.0);
    ros::param::param("~/LikelihoodGrid/grid_range_max",_globalGridFOV.range.max, 10.0);
    ros::param::param("~/LikelihoodGrid/grid_range_resolution",_globalGridFOV.range.resolution, 0.5);
    ROS_INFO("/LikelihoodGrid/grid_range min: %.2lf max: %.2lf: resolution: %.2lf",
             _globalGridFOV.range.min,
             _globalGridFOV.range.max,
             _globalGridFOV.range.resolution);

    double _update_rate;
    ros::param::param("~/LikelihoodGrid/update_rate",_update_rate, 0.5);
    ROS_INFO("/LikelihoodGrid/update_rate is set to %.2lf",_update_rate);

    double _free_cell_probability;
    ros::param::param("~/LikelihoodGrid/free_cell_probability",_free_cell_probability, 0.1);
    ROS_INFO("/LikelihoodGrid/free_cell_probability is set to %.2lf",_free_cell_probability);

    double _update_time_ratio;
    ros::param::param("~/update_time_ratio",_update_time_ratio, 10.0);
    ROS_INFO("/update_time_ratio is set to %.2lf",_update_time_ratio);

    // CREATE AN INSTANCE OF LIKELIHOOD GRID INTERFACE WITH PROPER PARAMETERS
    LikelihoodGridInterface lkGridInterface(n,
                                            _globalGridFOV,
                                            _update_rate,
                                            _update_time_ratio,
                                            _free_cell_probability);
    // FOR EVERY HUMAN FEATURE DATA (E.G. LEGS, FACES, SOUND)

    // DEFINE THE SENSOR FOV
    GridFOV_t legGridFOV = _globalGridFOV;
    GridFOV_t faceGridFOV = _globalGridFOV;
    faceGridFOV.range.min = 1.00;
    faceGridFOV.range.max = 5.00;
    faceGridFOV.angle.min = 0; // 0 degree
    faceGridFOV.angle.max = 1.0471975512; // 60 degree


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
        lkGridInterface.spin(2.0);
        ros::spinOnce();
        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long!");
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");
    }
    return 0;
}
