
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cleggrid.h"
#include <boost/bind.hpp>

#define _USE_MATH_DEFINES

using namespace message_filters;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_grid_node");
  ros::NodeHandle n;
  tf::TransformListener *tf_listener;
  int loop_rate;
  int probability_projection_step;
  ros::param::param("~/leg/probability_projection_step", probability_projection_step, 1);
  ros::param::param("~/loop_rate", loop_rate, 5);
  ros::Rate looprate(loop_rate);

  CLegGrid leg_grid(n, tf_listener, probability_projection_step);

//    message_filters::Subscriber<geometry_msgs::PoseArray> legs_sub(n, "legs", 10);
//    message_filters::Subscriber<nav_msgs::Odometry> encoder_sub(n, "encoder", 10);

//    typedef sync_policies::ApproximateTime <geometry_msgs::PoseArray,
//            nav_msgs::Odometry> MySyncPolicy;

//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), legs_sub, encoder_sub);
//    sync.registerCallback(boost::bind(&CLegGrid::syncCallBack,
//                                      &leg_grid, _1, _2));

  ros::Subscriber legs_sub = n.subscribe("legs", 10,
                                         &CLegGrid::legsCallBack, &leg_grid);
  ros::Subscriber encoder_sub = n.subscribe("husky/odom", 10, &CLegGrid::encoderCallBack, &leg_grid);

  while (ros::ok())
  {
    leg_grid.spin();

    if (looprate.cycleTime() > looprate.expectedCycleTime())
      ROS_ERROR("It is taking too long! %f", looprate.cycleTime().toSec());
    if (!looprate.sleep())
      ROS_ERROR("Not enough time left");

    ros::spinOnce();
  }

  return 0;
}
