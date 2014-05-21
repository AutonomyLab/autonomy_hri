#include <ros/ros.h>
#include "grid_interface.h"

#define _USE_MATH_DEFINES
#define _LOOPRATE 5

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    tf::TransformListener *tf_listener;
    int loop_rate;
    ros::param::param("~/loop_rate",loop_rate, 10);
    ros::Rate looprate(loop_rate);

    GridInterface likelihood_grid_interface(n,tf_listener);

    ros::Subscriber legs_sub = n.subscribe("legs",10,
                                           &GridInterface::legCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber faces_sub = n.subscribe("human",10,
                                           &GridInterface::faceCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber sound_sub = n.subscribe("HarkSource",10,
                                           &GridInterface::soundCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber laser_sub = n.subscribe("lidar/scan",10,
                                           &GridInterface::laserCallBack,
                                           &likelihood_grid_interface);
    while (ros::ok()) {

        likelihood_grid_interface.spin();
        ros::spinOnce();
        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");

    }
    return 0;
}
