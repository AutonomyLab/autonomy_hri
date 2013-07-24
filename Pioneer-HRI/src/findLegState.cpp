#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"

#define LASER_DATA 180
using namespace std;

void legsInLaserData(vector<float> ranges, int &numLegs, vector<float> &legPos){
    float diffLaserData[LASER_DATA];
    double threshold = 0.1;
    float filter[LASER_DATA];
    int i,laserIndex ,firstNeg, firstPos, secondNeg, secondPos, numLegTemp;
    numLegs = 0;
    const int legWidth = 15;

ROS_INFO("I got the data");
    // Initialize filter array with 1
    
     for (i = 0; i < LASER_DATA; i++){
        filter[i] = 1.00;
        diffLaserData[i] = 1.00;
     }
    
     /* Passing Laser Data from a filter: if the difference between two consecutive
      Laser Data will be more than the threshold, this quantity will be saved in
      * filter array, if not, the filter will be zero for that difference.
      * */
     for (int laserIndex = 0; laserIndex < (ranges.size()-1); laserIndex++){
         diffLaserData[laserIndex] = ranges.at(laserIndex+1) - ranges.at(laserIndex);
         if ((diffLaserData[laserIndex] < threshold) && (diffLaserData[laserIndex] > -threshold)){
             filter[laserIndex] = 0;
         } else {
             filter[laserIndex] = diffLaserData[laserIndex];
         }
     }
    
    //Searching for the legs:
    
     for (firstNeg = 0; firstNeg < (ranges.size()-1); firstNeg++){ 
         if (filter[firstNeg] < 0){
             
             for (firstPos = (firstNeg+1); (firstPos < (firstNeg+legWidth))&& (firstPos < (ranges.size()-1)); firstPos++){
                 if (filter[firstPos] > 0){ 
                     
                        for(secondNeg = (firstPos+1); (secondNeg < (firstPos+legWidth))&& (secondNeg < (ranges.size()-1)); secondNeg++){                            
                            if (filter[secondNeg] < 0){ //search for second negative difference
                                
                                for(secondPos = (secondNeg+1); (secondPos < (secondNeg+legWidth)) && (secondPos < (ranges.size()-1));secondPos++){
                                    if(filter[secondPos] > 0){ 
                                        legPos.push_back((firstPos + secondNeg)/2);
                                        numLegs++;
					firstNeg = secondPos;
					secondPos = secondNeg+legWidth;
					secondNeg = firstPos+legWidth;
					firstPos = ranges.size();
                                    } else if (filter[secondPos] < 0) break;
                                }
                            } else if (filter[secondNeg] > 0)break;
                        }
                 } else if (filter[firstPos] < 0) break;
             }
         }
     } // end of for on firstNeg
        
} //end of legsInLaserData function


void laserCallback(const sensor_msgs::LaserScan &msg)
{
 int numLegs;
 vector<float> ranges, legPos;
 msg.get_ranges_vec(ranges);
 legsInLaserData(ranges,numLegs,legPos);
 ROS_INFO("There are %d legs in robot's view (actually the size is %d)", numLegs, legPos.size());

 for (int i = 0; i < numLegs; i++){ 
    ROS_INFO("There is one at: %d, %6.4f", i, legPos.at(i));
 }
}

int main(int argc, char **argv)
{
 ros::init(argc,argv, "findLegState");
 ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("scan", 1, laserCallback); 
 ros::spin();
 return 0;
}

