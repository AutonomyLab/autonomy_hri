#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cartesian_grid_interface.h"
#include <boost/bind.hpp>

#define _USE_MATH_DEFINES
#define _LOOPRATE 5

using namespace message_filters;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"cartesian_likelihood_field");
    ros::NodeHandle n;
    tf::TransformListener *tf_listener;
    int loop_rate;
    ros::param::param("~/loop_rate",loop_rate, 10);
    ros::Rate looprate(loop_rate);

   CartesianGridInterface likelihood_grid_interface(n,tf_listener);

//    ros::Subscriber legs_sub = n.subscribe("legs",1,
//                                           &CartesianGridInterface::legCallBack,
//                                           &likelihood_grid_interface);

    ros::Subscriber faces_sub = n.subscribe("human",1,
                                           &CartesianGridInterface::faceCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber sound_sub = n.subscribe("HarkSource",1,
                                           &CartesianGridInterface::soundCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber laser_sub = n.subscribe("lidar/scan",1,
                                           &CartesianGridInterface::laserCallBack,
                                           &likelihood_grid_interface);
//    ros::Subscriber encoder_sub = n.subscribe("encoder",1,
//                                           &CartesianGridInterface::encoderCallBack,
//                                           &likelihood_grid_interface);

    message_filters::Subscriber<geometry_msgs::PoseArray> legs_sub(n, "legs", 1);
    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "encoder", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::Odometry> MySyncPolicy;
//    TimeSynchronizer<geometry_msgs::PoseArray, nav_msgs::Odometry> sync (legs_sub, encoder_sub, 10);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), legs_sub, encoder_sub);
    sync.registerCallback(boost::bind(&CartesianGridInterface::syncCallBack, &likelihood_grid_interface, _1, _2));

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
