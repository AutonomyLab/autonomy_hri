#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "p2os_driver/SonarArray.h"
#include "p2os_driver/GripperState.h"
// #include  "robot_params.h"
geometry_msgs::Twist cmd_vel;
p2os_driver::GripperState gripper_state;
p2os_driver::GripperState gripper_control;
bool danger = false;
bool wall = false;
bool gripper_cmd = false;
int last_gripper_cmd;

void sonarCallback(const p2os_driver::SonarArray &msg) {
    std::vector<double> ranges;
    msg.get_ranges_vec(ranges);



    // wall or not!

    if (ranges[11] < 0.75 || ranges[12] < 0.75) {
        wall = true;
    } else {
        wall = false;
    }


    if (((ranges[3] < 0.75) && (ranges[4] < 0.75)) && (danger == false) && (wall == false)) {
        cmd_vel.linear.x = -0.25;
        danger = true;
    } else if (((ranges[3] > 0.85) && (ranges[4] > 0.85)) && (danger == true)) {
        cmd_vel.linear.x = 0.0;
        danger = false;

    } else if (((ranges[3] < 0.75) && (ranges[4] < 0.75)) && (wall == true)) {
        cmd_vel.linear.x = 0.0;
    } else if (((ranges[3] > 0.75) || (ranges[4] > 0.75)) && (danger == false)) {
        cmd_vel.linear.x = 0.0;
    }

}

void gripper_cb(const p2os_driver::GripperState &msg){
    gripper_state = msg;
} // End of gripper_cb

void gripperGetObject() {
    if ((gripper_state.grip.left_contact) && (gripper_state.grip.right_contact) ) { // if Close
        if((!gripper_state.grip.outer_beam) && (!gripper_state.grip.inner_beam) ){ // WO object
            gripper_control.grip.state = 1;
            gripper_control.grip.dir = 1;
            gripper_cmd = true;
            if(last_gripper_cmd == 1){
                gripper_cmd = false;
            }
            last_gripper_cmd = gripper_control.grip.dir;
        } else { // With object
            gripper_cmd = false;
        }
    } else { // if Open
        if((!gripper_state.grip.outer_beam) && (!gripper_state.grip.inner_beam)){ // WO Object
            gripper_cmd = false;
        } else { // With object
                gripper_control.grip.state = 2;
                gripper_control.grip.dir = -1;
                gripper_cmd = true;
            if (last_gripper_cmd == -1) {
                gripper_cmd = false;
            }
            last_gripper_cmd = gripper_control.grip.dir;
        }
    }
    
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_base");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = cmd_vel.angular.z = 0.0;
    gripper_control = gripper_state;
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist > ("cmd_vel", 100);
    ros::Publisher gripper_pub = n.advertise<p2os_driver::GripperState > ("gripper_control", 50);
    ros::Subscriber gripper_sub = n.subscribe("gripper_state", 50, gripper_cb);
    ros::Subscriber sub = n.subscribe("sonar", 100, sonarCallback);
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        //   cmd_vel_pub_.publish(cmd_vel);
        gripperGetObject();
        if(gripper_cmd){
            gripper_pub.publish(gripper_control);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
