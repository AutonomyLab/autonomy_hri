
#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

//using namespace std
geometry_msgs::Twist cmd_vel;


void laser_cb(const sensor_msgs::LaserScan &msg)
{
 
 std::vector<float> ranges;
 msg.get_ranges_vec(ranges);
 float findLeg[180];
 int i, j, k, m,c;
 float diff[180];
 double threshold = 0.1;
 double error;
 
 //ROS_INFO("Number of ranges: [%d]", ranges.size());


 for (i = 0; i < 180; i++)
 {
     findLeg[i] = 1.00;
     diff[i] = 1.00;
 }
 
 
 
 for (i = 0; i < (ranges.size()-1); i++){
     diff[i] = ranges.at(i+1) - ranges.at(i);
          if ((diff[i] < threshold) && (diff[i] > -threshold)){
         findLeg[i] = 0;
     } 
     else {
         findLeg[i] = diff[i];
     }  
 }

// Calculating the difference vector
 for (i = 0; i < (ranges.size()-1); i++){

     if (findLeg[i] < 0){ //search for first negative difference
         for (j = (i+1); j < (i+15) ; j++){           
             if (j >= (ranges.size()-1)){
                 break;
             }
             if (findLeg[j] > 0){ //search for first positive difference after first positive difference
                 for(k = (j+1);k < (j+15); k++){
                     
                     if (k >= (ranges.size()-1)){
                        break;
                     }  
                     if (findLeg[k] < 0){ //search for second negative difference
                         for(m = (k+1); m < (k+15);m++){
                             if(m >= (ranges.size()-1)){
                                 break;
                             }
                             if(findLeg[m] > 0){ //search for second positive difference
                                 
                                 error = ((j+k)/2) - 90;
                                 cmd_vel.angular.z = ((1/(1+exp(-error/10.0))) - 0.5);

                                 if (fabs(error) < 4.0){
                                     ROS_INFO("Im in the while. error is : [%f]", error);
                                     cmd_vel.angular.z = 0.0; // stop rotating
                                     // find the nearest point of legs to the head of robot
                                     c = i+1;
                                 if (ranges.at(j) <= ranges.at(c)){
                                     c = j;
                                 }
                                  if (ranges.at(k+1) <= ranges.at(c)){
                                     c = k+1;
                                 }
                                 if (ranges.at(m) <= ranges.at(c)){
                                     c = m;
                                 }
                                ROS_INFO("i+1 : [%f]", ranges.at(i+1));   
				ROS_INFO("j : [%f]", ranges.at(j)); 
				ROS_INFO("k+1 : [%f]", ranges.at(k+1)); 
				 	if ( ((ranges.at(i+1) < 0.7) || (ranges.at(j) < 0.7) || (ranges.at(m) < 0.7)) || (fabs(error) > 20.0)){
					
						ROS_INFO("nearest point is : [%f]", ranges.at(c));
						cmd_vel.linear.x = 0.0;
                                                cmd_vel.angular.z = 0.0;
						break;
                                   }
                                     cmd_vel.linear.x = 0.25 * (1/(1+exp((ranges.at(c)-0.5)/15.0)));
                                 }
				
                             } else if (findLeg[m] < 0) {
                                 break;
                             }
                         }
                     } else if (findLeg[k] > 0){
                         break;
                     }
                 }
             } else if (findLeg[j] < 0){
                 break;
             }
         }
     }
 }


}


int main(int argc, char **argv)
{
 ros::init(argc,argv, "legDetection");
 cmd_vel.linear.y = cmd_vel.angular.z = cmd_vel.linear.x = 0.0;
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("scan", 100, laser_cb);
 ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
 ros::Rate loop_rate(1000);
  while (ros::ok()){  
   cmd_vel_pub_.publish(cmd_vel);
   ros::spinOnce();
   loop_rate.sleep();
 }
ros::spin();
 return 0;
}
