
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cvisiongrid.h"
#include <boost/bind.hpp>

#define _USE_MATH_DEFINES

using namespace message_filters;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"vision_grid_node");
    ros::NodeHandle n;
    tf::TransformListener *tf_listener;
    int loop_rate;
    ros::param::param("~/loop_rate",loop_rate, 5);
    ros::Rate looprate(loop_rate);

    CVisionGrid vision_grid(n, tf_listener);

    message_filters::Subscriber<autonomy_human::raw_detections> vision_sub(n, "torso", 10);
    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "encoder", 10);

    typedef sync_policies::ApproximateTime <autonomy_human::raw_detections,
            nav_msgs::Odometry> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vision_sub, encoder_sub);
    sync.registerCallback(boost::bind(&CVisionGrid::syncCallBack,
                                      &vision_grid, _1, _2));


    while (ros::ok())
    {
        vision_grid.spin();

        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("Sound Grid: It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_ERROR("Not enough time left");

        ros::spinOnce();
    }

    return 0;
}
