
#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

//using namespace std

void laser_cb(const sensor_msgs::LaserScan &msg)
{
 
 std::vector<float> ranges;
 msg.get_ranges_vec(ranges);
 ROS_INFO("Number of ranges: [%d]", ranges.size());
 int i = 90;
 for (i = 90; i = 145; i++){
 ROS_INFO("at %d degree: %f",i,ranges.at(i)); 
 
 if (i == 145){
     break;
 }
     
 }
 
// ROS_INFO("at 0 degree: %f",ranges.at(0));
// ROS_INFO("at 180 degree: %f",*ranges.end());
 ROS_INFO("Nearest point is: %f",*std::min_element(ranges.begin(),ranges.end()));
// std::min
 

}


int main(int argc, char **argv)
{
 ros::init(argc,argv, "findClosestPoint2Laser");
 
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("scan", 100, laser_cb);
 ros::spin();

 return 0;
}
