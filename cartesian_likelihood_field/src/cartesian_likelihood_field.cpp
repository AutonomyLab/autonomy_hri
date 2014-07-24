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

    message_filters::Subscriber<geometry_msgs::PoseArray> legs_sub(n, "legs", 1);
    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "encoder", 1);
    message_filters::Subscriber<autonomy_human::human> torso_sub(n, "human", 1);
    message_filters::Subscriber<hark_msgs::HarkSource> sound_sub(n, "HarkSource", 1);

    typedef sync_policies::ApproximateTime <geometry_msgs::PoseArray,
            nav_msgs::Odometry,
            autonomy_human::human,
            hark_msgs::HarkSource> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), legs_sub, encoder_sub, torso_sub, sound_sub);


//    typedef sync_policies::ApproximateTime <geometry_msgs::PoseArray,
//            nav_msgs::Odometry> MySyncPolicy;

//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), legs_sub, encoder_sub);

    sync.registerCallback(boost::bind(&CartesianGridInterface::syncCallBack,
                                      &likelihood_grid_interface, _1, _2, _3, _4));

//    sync.registerCallback(boost::bind(&CartesianGridInterface::syncCallBack,
//                                      &likelihood_grid_interface, _1, _2));

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
