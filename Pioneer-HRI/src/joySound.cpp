

#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
//#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sound_play/SoundRequest.h"

class VelSound
{
public:
	int play_button;
	bool play_, lplay_;
	std::string play_file;

	ros::NodeHandle n;
	ros::Subscriber vel_sub;
	ros::Publisher sound;
	sound_play::SoundRequest sound_req;

	JoySound(ros::NodeHandle &n) :
			n(n)
	{
	}

	void init(){

		n.param("play_button", play_button, 0);
		n.param<std::string>("play_file", play_file, "");

		sound = n.advertise<sound_play::SoundRequest>("/robotsound", 1, 0);

		ROS_INFO("play_button: %d", play_button);

		ROS_INFO("play_file: %s", play_file.c_str());

		if (!play_file.size())
			ROS_WARN("SOUND FILE HAS NOT BEEN SET! USE PARAM \"playfile\" FOR SETTING IT!");

		vel_sub = n.subscribe("/cmd_vel", 10, &VelSound::cmd_cb, this);

		sound_req.sound = sound_play::SoundRequest::PLAY_FILE;
		sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
		sound_req.arg = play_file;
	}

	~JoySound()
	{
	}

	void cmd_cb(const geometry_msgs::TwistConstPtr& cmd_msg){

		play_ = ((unsigned int) play_button < cmd_msg->angular.z);

		//Only execute the sound once on each button press
		if (play_){

			if (lplay_ == false){
				lplay_ = true;
				sound.publish(sound_req);
			}
		}
		else{
			lplay_ = false;
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joySound");
	ros::NodeHandle nh("~");
	VelSound sound(nh);
	sound.init();
	ros::spin();

	exit(0);
	return 0;
}

