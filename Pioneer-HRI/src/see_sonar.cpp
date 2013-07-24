#include "ros/ros.h"
#include "std_msgs/String.h"
#include "p2os_driver/SonarArray.h"
#include "sound_play/sound_play.h"
#include "sound_play/SoundRequest.h"
#include <string>
#include "ros/node_handle.h"
#include <unistd.h>

void sleepok(int t, ros::NodeHandle &n){
    if(n.ok()){
        sleep(t);
    }
}

void sonarCallback(const p2os_driver::SonarArray &msg){
 sound_play::SoundClient sc;
 std::vector<double> ranges; 
 msg.get_ranges_vec(ranges);
 ROS_INFO("%f",ranges[0]);
// while(n.ok()){
 if (ranges[0] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[1] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[2] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[3] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[4] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[5] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[6] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[7] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[8] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[9] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[10] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[11] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[12] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[13] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[14] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } else if (ranges[15] < 1.0){
     sc.playWave("/home/shokoofeh/ros/ros_workspace/RobotSounds/A/AError1.wav");
        ROS_INFO("Say: Hello World...");
        sleep(1);
 } ;
 }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "see_sonar");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sonar", 1000, sonarCallback);
    ros::spin();
    return 0;
}