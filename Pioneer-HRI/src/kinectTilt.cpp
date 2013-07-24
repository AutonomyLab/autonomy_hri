#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"

using namespace std;

vector<float> headPos;
vector<float> cur_angle;
std_msgs::Float64 tilt_angle;

void face_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    headPos.push_back(msg->point.y);
   }

void cur_angle_cb(const std_msgs::Float64::ConstPtr& msg){
    cur_angle.push_back(msg->data);
    if (cur_angle.back() < -40.0){
        ROS_ERROR("It is tilting!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinectTilt");
    ros::NodeHandle n;
    float desirePos = -0.1;
    float error;
    float control_command;
    float max_angle = 30.0;
    float min_angle = -30.0;
    bool tilt_init = true;
    
    ros::Subscriber sub_face = n.subscribe("target_point", 100, face_cb);
    ros::Subscriber sub_cur_angle = n.subscribe("cur_tilt_angle", 100, cur_angle_cb);
    ros::Publisher tilt_pub = n.advertise<std_msgs::Float64>("tilt_angle", 100);


    ros::Rate loop_rate(10);
    while (ros::ok()){
        if(!cur_angle.empty()){
            if(tilt_init && cur_angle.back() != 0.0){
                    tilt_angle.data = 0.0;
                    tilt_pub.publish(tilt_angle);
                    tilt_init = false;
            }
        }
        
        if(! headPos.empty() && !cur_angle.empty() && cur_angle.back() > -40.0){
            
                error = desirePos - headPos.back();
                ROS_INFO("Error: %f", error);
                
                if (fabs(error) > 0.01){
                        if (error > 0.0)
                                control_command = ((max_angle - cur_angle.back())/2) * error;
                        else 
                                control_command = -((min_angle - cur_angle.back())/2) * error;
                } 
                else control_command = 0.0;
                
                ROS_INFO("Control Command is %f", control_command);
                
                tilt_angle.data = cur_angle.back() + control_command; 
                if(tilt_angle.data > 29.0) tilt_angle.data = 29.0;
                else if(tilt_angle.data < -29.0) tilt_angle.data = -29.0;   
                ROS_INFO("Tilt command: %f", tilt_angle.data);
                tilt_pub.publish(tilt_angle);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;          
}