
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sound_play/sound_play.h"
#include "sound_play/SoundRequest.h"
using namespace std;
sound_play::SoundRequest robotsound;

// %Tag(CALLBACK)%
void cmdvelCallback(const geometry_msgs::TwistConstPtr & msg)
{
    ROS_INFO("velocity is: [%6.4f]", msg->angular.z);
    sound_play::SoundClient sc;
    sleep(1);
    sc.playWave("/home/autolab/ros/ros_workspace/RobotSounds/I/IDiag2.wav");
    sleep(3);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "soundWalk");
  ros::NodeHandle n;
  ros::Rate loopRate(0.3);
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdvelCallback);
//  ros::spin();
  ros::Publisher robotsound_pub_ = n.advertise<sound_play::SoundRequest>("/robotsound",10);
 while(ros::ok()){
     robotsound_pub_.publish(robotsound);
     ros::spinOnce();
     loopRate.sleep();
 }
  return 0;
}
// %EndTag(FULLTEXT)%
