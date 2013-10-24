#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "pi_tracker/Skeleton.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include <deque>

using namespace std;
#define JOINTS_NUM 15
deque<pi_tracker::Skeleton> past_persons;
ros::Publisher pub_gesture;
void mean_p(deque<pi_tracker::Skeleton>, pi_tracker::Skeleton &);
void person_edges(pi_tracker::Skeleton, float &, float &, float &, float &);
void hand_gesture(pi_tracker::Skeleton);
float sr = 1.00; // Sphere Radius
geometry_msgs::Point sphereCenter;

bool reachingGesture(const geometry_msgs::Vector3 &p1, const geometry_msgs::Vector3 &p2)
{
    // Joint data:  [1:'head', 2:'neck', 3:'torso', 4:'left_shoulder', 5:'left_elbow',
    //               6:'left_hand', 7:'right_shoulder', 8:'right_elbow', 9:'right_hand', 10:'left_hip',
    //               11:'left_knee', 12:'left_foot', 13:'right_hip', 14:'right_knee', 15:'right_foot']
    geometry_msgs::Point  p3; // sphere center
    p3.x = 0.0;
    p3.y = 0.0;
    p3.z = 0.0;
    float  a,b,c,delta;

    /* Calculating the line-sphere intersection:
      P1 (x1,y1,z1), P2 (x2,y2,z2) : line points
      P3 (x3,y3,z3) : sphere center
      r : sphere radius
      a = (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2;
      b = 2*((x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3));
      c = x3^2 + y3^2 + z3^2 + x1^2 + y1^2 + z1^2 - 2*(x3*x1 + y3*y1 + z3*z1) - r^2;
      delta = b * b - 4 * a * c;
      */

      a = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) + (p2.z - p1.z)*(p2.z - p1.z);
      b = 2*((p2.x - p1.x)*(p1.x - p3.x) + (p2.y - p1.y)*(p1.y - p3.y) + (p2.z - p1.z)*(p1.z - p3.z));
      c = p3.x * p3.x + p3.y * p3.y + p3.z*p3.z + p1.x*p1.x + p1.y*p1.y + p1.z*p1.z - 2*(p3.x*p1.x + p3.y*p1.y + p3.z*p1.z) - sr*sr;
      delta = b * b - 4 * a * c;

    if(delta >= 0.0)
    {
        ROS_INFO("              *** REACHING GESTURE DETECTED  ***");
        return true;
    }
    else

        return false;
}


void skeleton_cb(const pi_tracker::Skeleton &msg)
{
    pi_tracker::Skeleton Persons, mean_person;
    bool allConf = true;
    Persons = msg;
    vector<string> Names;
    unsigned int counter = 10;
    Names = Persons.name;
    float depth_min, depth_max, height_min, height_max;
    if (Persons.confidence.size() > 0) {
        //allConf = true;
        //if (allConf) {

            if (past_persons.size() < counter) {
                past_persons.push_back(Persons);
            } else {
                past_persons.pop_front();
                past_persons.push_back(Persons);
            }
        //}
        if (past_persons.size() == counter) {
            mean_p(past_persons, mean_person);
            person_edges(mean_person, depth_min, depth_max, height_min, height_max);
            hand_gesture(mean_person);
        }
    }
} // End of skeleton_cb

void mean_p(deque<pi_tracker::Skeleton> past_persons, pi_tracker::Skeleton & mean_person)
{

    int mean_size = past_persons.size(); // 10
    mean_person.confidence.resize(15.0);
    mean_person.name.resize(15.0);
    mean_person.orientation.resize(15.0);
    mean_person.position.resize(15.0);
    
    pi_tracker::Skeleton t_person = past_persons.at(0);
    
    vector<string> joint_names;
    //t_person.get_name_vec(joint_names);
    joint_names = t_person.name;

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

void hand_gesture(pi_tracker::Skeleton mean_person)
{

    geometry_msgs::Vector3  rightHand, head,leftHand;
    float d_min, d_max, h_min, h_max;
    string gest;
    std_msgs::String msg_gesture;
    stringstream str_gesture;
    person_edges(mean_person, d_min, d_max, h_min, h_max);
   /* float r_hand_sh2el, r_hand_el2ha, r_hand_ha2hip, l_hand_sh2el, l_hand_el2ha, l_hand_ha2hip; // different between joints
    r_hand_sh2el = mean_person.position.at(6).y - mean_person.position.at(7).y;
    r_hand_el2ha = mean_person.position.at(7).y - mean_person.position.at(8).y;
    r_hand_ha2hip = mean_person.position.at(8).y - mean_person.position.at(12).y;
    l_hand_sh2el = mean_person.position.at(3).y - mean_person.position.at(4).y;
    l_hand_el2ha = mean_person.position.at(4).y - mean_person.position.at(5).y;
    l_hand_ha2hip = mean_person.position.at(5).y - mean_person.position.at(9).y; */

    rightHand = mean_person.position.at(8);
    leftHand = mean_person.position.at(5);
    head = mean_person.position.at(0);
    if(reachingGesture(head,rightHand))
    {
        ROS_INFO("              *** REACHING GESTURE DETECTED (RIGHT HAND) ***");
        gest = "reach right";
    }
    else if(reachingGesture(head,leftHand))
    {
        ROS_INFO("              *** REACHING GESTURE DETECTED (Left HAND) ***");
        gest = "left right";
    }



/*    if ((fabs(d_min - mean_person.position.at(8).z) < 0.01) && (r_hand_sh2el < l_hand_sh2el) && (r_hand_el2ha < l_hand_el2ha) && (r_hand_ha2hip > 0)) {
        ROS_INFO("              *** RIGHT HAND GESTURE DETECTED ***");
        gest = "right";
    }

    if ((fabs(d_min - mean_person.position.at(5).z) < 0.01) && (r_hand_sh2el > l_hand_sh2el) && (r_hand_el2ha > l_hand_el2ha) && (l_hand_ha2hip > 0)) {
        ROS_INFO("              *** Left HAND GESTURE DETECTED ***");
        gest = "left";
    }
*/
    str_gesture << gest;
    msg_gesture.data = str_gesture.str();
    pub_gesture.publish(msg_gesture);
}

void person_edges(pi_tracker::Skeleton person, float & depth_min, float & depth_max, float & height_min, float & height_max)
{
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

int main(int argc, char **argv)
{

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
