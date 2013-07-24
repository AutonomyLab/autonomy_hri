#include "ros/ros.h"
#include "sound_play/sound_play.h"
#include "sound_play/SoundRequest.h"
#include <string>
#include "ros/node_handle.h"
#include <unistd.h>
#include "pi_tracker/Skeleton.h"
//sound_play::SoundRequest robotsound;

void sleepok(int t, ros::NodeHandle &n){
    if(n.ok()){
        sleep(t);
    }
}
void skeleton_cb(const pi_tracker::Skeleton &msg)
{
    int isUser;
    
//    msg.get_position_size(isUser);
     std::vector<float> ranges;
//     msg.get_position_vec(ranges);
     isUser = msg.user_id;
    
    if(isUser){
        ROS_INFO("I can see you %d ", isUser );
    }else {
        
    }
    
}

int main(int argc, char **argv){
        ros::init(argc,argv, "justTalk");
        ros::NodeHandle n;
        ROS_INFO("No body is here! %d");
        ros::Subscriber skeleton_sub = n.subscribe("skeleton", 100, skeleton_cb);
        ros::spin();
//        sound_play::SoundClient sc;
//        sleepok(1,n);
//        ros::Publisher robotsound_pub_ = n.advertise<sound_play::SoundRequest>("/robotsound",10);

//while(n.ok()){
////    sc.playWave("/home/autolab/ros/ros_workspace/RobotSounds/A/AError1.wav");
////    sleepok(3,n);
//}
 return 0;
}