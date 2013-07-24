
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

#define LASER_DATA 180
#define LASER_MAX_VIEW 8.0
using namespace std;

void laser_cb(const sensor_msgs::LaserScan &msg){ 
 std::vector<float> ranges;
 msg.get_ranges_vec(ranges);
float diffLaserData[LASER_DATA], filter[LASER_DATA];
double threshold = 0.1;
 
 for (int i = 0; i < LASER_DATA; i++){
        filter[i] = 1.00;
        diffLaserData[i] = 1.00;
        if(ranges.at(i) >= LASER_MAX_VIEW) ranges.at(i) = LASER_MAX_VIEW;
     }

for (int laserIndex = 0; laserIndex < (ranges.size()-1); laserIndex++){
         diffLaserData[laserIndex] = ranges.at(laserIndex+1) - ranges.at(laserIndex);
         if ((diffLaserData[laserIndex] < threshold) && (diffLaserData[laserIndex] > -threshold)){
             filter[laserIndex] = 0;
         } else {
             filter[laserIndex] = diffLaserData[laserIndex];
         }
     }
for(int k = 0; k < 179; k++){
    if((filter[k]-filter[k+1] > 7.0) || (filter[k]-filter[k+1] < -7.0)){
        filter[k+1] = 0.55555;
    }
    
}
for(int i=70; i<110; i++){
//     if(ranges.at(i) >= 8.0) ranges.at(i) = 8.0;
     ROS_INFO("at %d degree: %f",i, ranges[i]);
 }

} // end of laserCallback function


int main(int argc, char **argv)
{
 ros::init(argc,argv, "see_laser");
 
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("scan", 100, laser_cb);
 ros::spin();

 return 0;
}
