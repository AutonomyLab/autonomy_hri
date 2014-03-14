#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <algorithm>
#include <assert.h>

#define LASER_MAX_RANGE 8.0
#define FILTER_THRESHOLD 0.1
#define LEG_WIDTH 10
#define PI 3.1415

struct leg_t{
    float x;
    float y;
    float z;
};

ros::Publisher leg_pub;


template<typename T>
void vector_derivative(const std::vector<T> &src,
                       std::vector<T> &dest)
{
    assert(dest.size() <= src.size());
    for (uint8_t i = 0; i < (src.size()-1); i++) {
        dest.at(i) = src.at(i+1)-src.at(i);
    }
}

template<typename T>
T min_cut(const T val,
          const T thresh,
          const T cut)
{
    return((val<thresh)?cut:val);
}

template<typename T>
T max_cut(const T val,
          const T thresh,
          const T cut)
{
    return((val>thresh)?cut:val);
}

bool IsLeg(const std::vector<float> &ranges,
           uint8_t start1,
           uint8_t stop1,
           uint8_t start2,
           uint8_t stop2)
{
    float it1 = *std::min_element(ranges.begin()+start1,ranges.begin()+stop1);
    float it2 = *std::min_element(ranges.begin()+start2,ranges.begin()+stop2);
    return !(fabs(it1 - it2) > 1.00);
}

void filter_laser(const std::vector<float> &ranges,
                 std::vector<float> &filterRanges,
                  const float threshold = 0.1)
{

    /* Passing Laser Data from a filter: if the difference between two consecutive
     Laser Data will be more than the threshold, this quantity will be saved in
     filter array, if not, the filter will be zero for that difference.
     * */

    assert(filterRanges.empty());
    assert(!ranges.empty());
    filterRanges = ranges;
    for (uint8_t i = 0; i < (ranges.size()); i++) {
        if (ranges.at(i) > LASER_MAX_RANGE) filterRanges.at(i) = LASER_MAX_RANGE;
    }

    vector_derivative(ranges,filterRanges);

    for (uint8_t j = 0; j < (filterRanges.size()); j++) {
        if(fabs(filterRanges.at(j)) < threshold ){
            filterRanges.at(j) = 0.0;
        }
        /*
        if (fabs(ranges.at(j+1)-ranges.at(j))<threshold) {
            filterRanges.push_back(0.0);
        } else {
            filterRanges.push_back(ranges.at(j+1)-ranges.at(j));
        }*/
    }

    /*filter out some noises*/
    std::vector<float> temp;
    temp = filterRanges;
    vector_derivative(filterRanges,temp);
    for(uint8_t k=0;k<filterRanges.size()-1;k++){
        if(fabs(temp.at(k))>LASER_MAX_RANGE)
            filterRanges.at(k+1) = 0.0;
    }
}

void detect_leg(const std::vector<float> &ranges,
                std::vector<leg_t> &legLaser,
                uint8_t startRange,
                uint8_t endRange)
{
    if(ranges.empty()) return;
    uint8_t firstNeg, firstPos, secondNeg, secondPos;
    uint8_t LegWidth = 10;
    float preLegAng;
    std::vector<float> filter;
    filter_laser(ranges,filter,FILTER_THRESHOLD);
    if(endRange > filter.size()) endRange = filter.size();
    firstNeg = startRange;
    for(int i = 0;i<filter.size();i++)
         if (filter.at(i) != 0.0)

    while (firstNeg < endRange) {
        if (filter.at(firstNeg) < 0.0) {
            firstPos = firstNeg + 1;
            while ((firstPos < (firstNeg + LegWidth)) && (firstPos < endRange)) {
                if (filter.at(firstPos) > 0.0) {
                    secondNeg = firstPos + 1;
                    while ((secondNeg < (firstPos + LegWidth)) && (secondNeg < endRange)) {
                        if (filter.at(secondNeg) < 0.0) {
                            secondPos = secondNeg + 1;
                            while ((secondPos < (secondNeg + LegWidth)) && (secondPos < endRange)) {
                                if (filter.at(secondPos) > 0.0) {
                                    if (!IsLeg(ranges, firstNeg, firstPos, secondNeg, secondPos)) break;
                                    //ROS_INFO("I think I saw a leg: %d",legLaser.size());

                                    float ang2Leg = (firstPos + secondNeg) / 2;
                                    //nearestPoint(ranges, firstNeg, secondPos, NearestDist, nearestP);
                                    float dist2Leg = *std::min_element(ranges.begin()+firstNeg,ranges.begin()+secondPos);
                                    leg_t tempLeg;

                                    if (legLaser.empty()) {// add first leg
                                        //tempLeg.distance = dist2Leg;
                                        //tempLeg.theta = ang2Leg;
                                        tempLeg.x = dist2Leg*sin(ang2Leg*PI/180); // In the ROS Coordinate frame
                                        tempLeg.y = -dist2Leg*cos(ang2Leg*PI/180); // In the ROS Coordinate frame
                                        legLaser.push_back(tempLeg);
                                        preLegAng = ang2Leg;
                                    } else{ // add more legs if they are different from the first pairs of legs
                                        if (ang2Leg != preLegAng || legLaser.size() == 1) {
                                            //tempLeg.distance = dist2Leg;
                                            //tempLeg.theta = ang2Leg;
                                            tempLeg.x = dist2Leg*sin(ang2Leg*PI/180); // In the ROS Coordinate frame
                                            tempLeg.y = -dist2Leg*cos(ang2Leg*PI/180); // In the ROS Coordinate frame
                                            legLaser.push_back(tempLeg);
                                            preLegAng = ang2Leg;
                                        }
                                    }
                                    firstNeg = secondPos;
                                    secondNeg = endRange;
                                    firstPos = endRange;
                                    break;
                                }/*end of if(filter[secondPos] > 0)*/
                                ++secondPos;
                            }/* end of while on secondPos */
                        } /* end of  if (filter[secondNeg] < 0) */
                        ++secondNeg;
                    }/*end of while on secondNeg */
                }/*end of if (filter[firstPos] > 0) */
                ++firstPos;
            } /* end while on firstPos */
        } /* end of if (filter[firstNeg] < 0) */
        ++firstNeg;
    } /* end while on firstNeg */

} /* end of detect_leg function */

void laser_cb(const sensor_msgs::LaserScan & msg)
{
    ROS_INFO("LASER DATA CAME");
    std::vector<float> ranges;
    ranges.clear();
    if (!msg.ranges.empty()) {
        ranges = msg.ranges;
        std::vector<leg_t> legLaser;
        legLaser.clear();
        detect_leg(ranges,legLaser,0,180);
        geometry_msgs::PoseArray legs;
        geometry_msgs::Pose legPoses;
        legs.header.frame_id = msg.header.frame_id;
        legs.header.stamp = ros::Time::now();
        //ROS_INFO("Number of legs: %d",legLaser.size());
        for(uint32_t i = 0; i < legLaser.size(); i++){
            legPoses.position.x = legLaser.at(i).x;
            legPoses.position.y = legLaser.at(i).y;
            legPoses.position.z = 0.0;
            legs.poses.push_back(legPoses);
        }
        leg_pub.publish(legs);
        /*
         * //Plot laser data
         * clearVisWindow();
        for (unsigned int i = 0; i < laser_ranges.size(); i++) {
            insertPoint(laser_ranges.at(i) * 1000.0, (float) i, CV_RGB(255, 255, 255));
        }
        */
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"leg_detection");
    ros::NodeHandle n;
    ROS_INFO("Starting Leg Detection ...");
    ros:: Rate loopRate(30);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, laser_cb);
    leg_pub = n.advertise<geometry_msgs::PoseArray>("legs",10);

    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
