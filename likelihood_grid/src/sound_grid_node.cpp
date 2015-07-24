#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "csoundgrid.h"
#include <boost/bind.hpp>

#define _USE_MATH_DEFINES

using namespace message_filters;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sound_grid_node");
    ros::NodeHandle n;
    tf::TransformListener *tf_listener;
    int loop_rate;
    int probability_projection_step;
    ros::param::param("~/loop_rate",loop_rate, 5);
    ros::param::param("~/sound/probability_projection_step",probability_projection_step , 1);
    ros::Rate looprate(loop_rate);

    CSoundGrid sound_grid(n, tf_listener, probability_projection_step);

    message_filters::Subscriber<hark_msgs::HarkSource> sound_sub(n, "sound", 10);
    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "husky/odom", 10);

    typedef sync_policies::ApproximateTime <hark_msgs::HarkSource,
            nav_msgs::Odometry> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sound_sub, encoder_sub);
    sync.registerCallback(boost::bind(&CSoundGrid::syncCallBack,
                                      &sound_grid, _1, _2));


    while (ros::ok())
    {
        sound_grid.spin();

        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("Sound Grid: It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_ERROR("Not enough time left");

        ros::spinOnce();
    }

    return 0;
}
