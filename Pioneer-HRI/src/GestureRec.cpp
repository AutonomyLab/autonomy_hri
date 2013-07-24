#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "pi_tracker/Skeleton.h"
#include "geometry_msgs/Twist.h"
#include <deque>

using namespace std;
#define JOINTS_NUM 15
deque<pi_tracker::Skeleton> past_persons;
ros::Publisher pub_gesture;
void mean_p(deque<pi_tracker::Skeleton>, pi_tracker::Skeleton &);
void person_edges(pi_tracker::Skeleton, float &, float &, float &, float &);
void hand_gesture(pi_tracker::Skeleton);

void skeleton_cb(const pi_tracker::Skeleton &msg) {
    pi_tracker::Skeleton Persons, mean_person;
    bool allConf = true;
    Persons = msg;
    vector<string> Names;
    int counter = 10;
    Persons.get_name_vec(Names);
    float depth_min, depth_max, height_min, height_max;
    if (Persons.get_confidence_size() > 0) {
//        for (int i = 0; i < Persons.get_confidence_size(); i++) {
//            allConf = ((Persons.confidence.at(i) == 1) && allConf);
//        }
        //            ROS_INFO("allConf: %d",allConf);
        allConf = true;
        if (allConf) {

            if (past_persons.size() < counter) {
                past_persons.push_back(Persons);
            } else {
                past_persons.pop_front();
                past_persons.push_back(Persons);
            }
        }
        if (past_persons.size() == counter) {
            mean_p(past_persons, mean_person);
            person_edges(mean_person, depth_min, depth_max, height_min, height_max);
            hand_gesture(mean_person);
        }
    }
} // End of skeleton_cb

void mean_p(deque<pi_tracker::Skeleton> past_persons, pi_tracker::Skeleton & mean_person) {

    int mean_size = past_persons.size(); // 10
    mean_person.set_confidence_size(15);
    mean_person.set_name_size(15);
    mean_person.set_orientation_size(15);
    mean_person.set_position_size(15);
    
    pi_tracker::Skeleton t_person = past_persons.at(0);
    
    vector<string> joint_names;
    t_person.get_name_vec(joint_names);
    for (int i = 0; i < JOINTS_NUM; i++) {
        mean_person.position.at(i).x = 0.0;
        mean_person.position.at(i).y = 0.0;
        mean_person.position.at(i).z = 0.0;
        mean_person.name.at(i) = joint_names.at(i);
    }

    for (int i = 1; i < mean_size; i++) { // for on number of person
        pi_tracker::Skeleton temp_person = past_persons.at(i);
        for (int j = 0; j < JOINTS_NUM; j++) { // for on number of joints
            mean_person.position.at(j).x = mean_person.position.at(j).x + temp_person.position.at(j).x;
            mean_person.position.at(j).y = mean_person.position.at(j).y + temp_person.position.at(j).y;
            mean_person.position.at(j).z = mean_person.position.at(j).z + temp_person.position.at(j).z;
        }
    }
    for (int i = 0; i < JOINTS_NUM; i++) {
        mean_person.position.at(i).x = (mean_person.position.at(i).x / mean_size);
        mean_person.position.at(i).y = (mean_person.position.at(i).y / mean_size);
        mean_person.position.at(i).z = (mean_person.position.at(i).z / mean_size);
    }
}

void hand_gesture(pi_tracker::Skeleton mean_person) {

    float d_min, d_max, h_min, h_max;
    string gest;
    std_msgs::String msg_gesture;
    stringstream str_gesture;
    person_edges(mean_person, d_min, d_max, h_min, h_max);
    float r_hand_sh2el, r_hand_el2ha, r_hand_ha2hip, l_hand_sh2el, l_hand_el2ha, l_hand_ha2hip; // different between joints
    r_hand_sh2el = mean_person.position.at(6).y - mean_person.position.at(7).y;
    r_hand_el2ha = mean_person.position.at(7).y - mean_person.position.at(8).y;
    r_hand_ha2hip = mean_person.position.at(8).y - mean_person.position.at(12).y;
    l_hand_sh2el = mean_person.position.at(3).y - mean_person.position.at(4).y;
    l_hand_el2ha = mean_person.position.at(4).y - mean_person.position.at(5).y;
    l_hand_ha2hip = mean_person.position.at(5).y - mean_person.position.at(9).y;

    if ((fabs(d_min - mean_person.position.at(8).z) < 0.01) && (r_hand_sh2el < l_hand_sh2el) && (r_hand_el2ha < l_hand_el2ha) && (r_hand_ha2hip > 0)) {
        ROS_INFO("              *** RIGHT HAND GESTURE DETECTED ***");
        gest = "right";
    }

    if ((fabs(d_min - mean_person.position.at(5).z) < 0.01) && (r_hand_sh2el > l_hand_sh2el) && (r_hand_el2ha > l_hand_el2ha) && (l_hand_ha2hip > 0)) {
        ROS_INFO("              *** Left HAND GESTURE DETECTED ***");
        gest = "left";
    }
    str_gesture << gest;
    msg_gesture.data = str_gesture.str();
    pub_gesture.publish(msg_gesture);
}

void person_edges(pi_tracker::Skeleton person, float & depth_min, float & depth_max, float & height_min, float & height_max) {
    depth_min = person.position.at(0).z;
    depth_max = person.position.at(0).z;
    height_min = person.position.at(0).y;
    height_max = person.position.at(0).y;
    for (int i = 0; i < JOINTS_NUM; i++) {
        if (person.position.at(i).z < depth_min) depth_min = person.position.at(i).z;
        if (person.position.at(i).z > depth_max) depth_max = person.position.at(i).z;
        if (person.position.at(i).y < height_min) height_min = person.position.at(i).y;
        if (person.position.at(i).y > height_max) height_max = person.position.at(i).y;
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "GestureRec");
    ros::NodeHandle n;

    ros::Subscriber sub_skel = n.subscribe("skeleton", 100, skeleton_cb);
    pub_gesture = n.advertise<std_msgs::String>("gesture", 100);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}