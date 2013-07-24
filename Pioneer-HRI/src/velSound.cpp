

#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
//#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sound_play/SoundRequest.h"


sound_play::SoundRequest sound_mv;
ros::Publisher sound_pub;
std::string play_file;
int counter = 0;

 void cmd_cb(const geometry_msgs::TwistConstPtr& cmd_msg){
//        ROS_INFO("Velocity is: %f", cmd_msg->angular.z);
        
        //Only execute the sound once on each button press
        if (cmd_msg->angular.z != 0 && counter > 200 ){
            ROS_INFO("Salaaaaaam");
            sound_mv.arg = play_file;
            sound_pub.publish(sound_mv);
            counter = 1;
        }
counter++ ;
    }
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velSound");
	ros::NodeHandle nh;
        ros::Rate loopRate(30);
        sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound",10);
        nh.param<std::string>("play_file", play_file, "/home/autolab/ros/ros_workspace/RobotSounds/A/AHey.ogg");
        if (!play_file.size())
                ROS_WARN("SOUND FILE HAS NOT BEEN SET! USE PARAM \"playfile\" FOR SETTING IT!");
        ROS_INFO("play_file: %s", play_file.c_str());
        sound_mv.sound = sound_play::SoundRequest::PLAY_FILE;
        sound_mv.command = sound_play::SoundRequest::PLAY_ONCE;
        ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 5, cmd_cb);
        loopRate.sleep();
        ros::spin();

	return 0;
}