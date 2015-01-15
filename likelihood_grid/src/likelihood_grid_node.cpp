#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "likelihood_grid.h"
#include <boost/bind.hpp>

#define _USE_MATH_DEFINES
#define _LOOPRATE 5

using namespace message_filters;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_grid_node");
    ros::NodeHandle n;
    tf::TransformListener *tf_listener;
    int loop_rate;
    ros::param::param("~/loop_rate",loop_rate, 20);
    ros::Rate looprate(loop_rate);

    LikelihoodGrid likelihood_grid_interface(n,tf_listener);

    message_filters::Subscriber<geometry_msgs::PoseArray> legs_sub(n, "legs", 10);
    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "encoder", 10);

    typedef sync_policies::ApproximateTime <geometry_msgs::PoseArray,
            nav_msgs::Odometry> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), legs_sub, encoder_sub);
    sync.registerCallback(boost::bind(&LikelihoodGrid::syncCallBack,
                                      &likelihood_grid_interface, _1, _2));

    ros::Subscriber sound_sub = n.subscribe("HarkSource",10,
                                           &LikelihoodGrid::soundCallBack,
                                           &likelihood_grid_interface);

    ros::Subscriber torso_sub = n.subscribe("torso", 10,
                                            &LikelihoodGrid::torsoCallBack,
                                            &likelihood_grid_interface);

    ros::Subscriber periodic_sub = n.subscribe("periodic_gestures/raw_detections", 10,
                                            &LikelihoodGrid::periodicCallBack,
                                            &likelihood_grid_interface);

    while (ros::ok())
    {
        likelihood_grid_interface.spin();
        ros::spinOnce();

        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");
    }
    return 0;
}
