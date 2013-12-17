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
#define PI 3.1415
deque<pi_tracker::Skeleton> past_persons;
ros::Publisher pub_gesture;
void mean_p(deque<pi_tracker::Skeleton>, pi_tracker::Skeleton &);
//void person_edges(pi_tracker::Skeleton, float &, float &, float &, float &);
void hand_gesture(pi_tracker::Skeleton);
float sr = 1.50; // Sphere Radius
geometry_msgs::Point sphereCenter;
string gest,last_gest;
unsigned int gest_counter;
ros::Time last_skel;
std_msgs::String no_gesture;


bool reachingGesture(const geometry_msgs::Vector3 &p1, const geometry_msgs::Vector3 &p2)
{
    /*
      Joint data:  [1:'head', 2:'neck', 3:'torso', 4:'left_shoulder', 5:'left_elbow',
                    6:'left_hand', 7:'right_shoulder', 8:'right_elbow', 9:'right_hand', 10:'left_hip',
                    11:'left_knee', 12:'left_foot', 13:'right_hip', 14:'right_knee', 15:'right_foot']

      Calculating the line-sphere intersection:
      P1 (x1,y1,z1), P2 (x2,y2,z2) : line points
      P3 (x3,y3,z3) : sphere center
      r : sphere radius
      a = (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2;
      b = 2*((x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3));
      c = x3^2 + y3^2 + z3^2 + x1^2 + y1^2 + z1^2 - 2*(x3*x1 + y3*y1 + z3*z1) - r^2;
      delta = b * b - 4 * a * c;
      */

    geometry_msgs::Point  p3; // sphere center
    p3.x = 0.0;
    p3.y = 0.0;
    p3.z = 0.0;
    float  a,b,c,delta;
      a = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) + (p2.z - p1.z)*(p2.z - p1.z);
      b = 2*((p2.x - p1.x)*(p1.x - p3.x) + (p2.y - p1.y)*(p1.y - p3.y) + (p2.z - p1.z)*(p1.z - p3.z));
      c = p3.x * p3.x + p3.y * p3.y + p3.z*p3.z + p1.x*p1.x + p1.y*p1.y + p1.z*p1.z - 2*(p3.x*p1.x + p3.y*p1.y + p3.z*p1.z) - sr*sr;
      delta = b * b - 4 * a * c;

    if(delta >= 0.0)
    {
        ROS_INFO("***** REACHING GESTURE DETECTED  *****");
        return true;
    }
    else

        return false;
}

void slope(const geometry_msgs::Vector3 &p1, const geometry_msgs::Vector3 &p2, float &alpha, float &beta, float &gama)
{
    float x = fabs(p1.x - p2.x);
    float y = fabs(p1.y - p2.y);
    float z = fabs(p1.z - p2.z);
    float lineSize = sqrt(x*x + y*y + z*z);
    alpha = acos(x/lineSize)*180.0/PI;
    beta = acos(y/lineSize)*180.0/PI;
    gama = acos(z/lineSize)*180.0/PI;
}


void skeleton_cb(const pi_tracker::Skeleton &msg)
{
    //last_skel = msg.header.stamp;
    last_skel = ros::Time::now();
    pi_tracker::Skeleton Persons, mean_person;
    bool allConf = true;
    Persons = msg;
    vector<string> Names;
    unsigned int counter = 10;
    Names = Persons.name;
    //float depth_min, depth_max, height_min, height_max;
    for(unsigned int i = 0; i < (JOINTS_NUM - 6) ; i++)
    {
        allConf = allConf && Persons.confidence.at(i);
    }
    if (Persons.confidence.size() > 0) {
        //ROS_INFO("CONFIDENCE? [%d]",allConf);
        if (allConf) {

            if (past_persons.size() < counter) {
                past_persons.push_back(Persons);
            } else {
                past_persons.pop_front();
                past_persons.push_back(Persons);
            }

            if (past_persons.size() == counter) {
                mean_p(past_persons, mean_person);
                //person_edges(mean_person, depth_min, depth_max, height_min, height_max);
                hand_gesture(mean_person);
            }
        }
        else
            pub_gesture.publish(no_gesture);
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

    geometry_msgs::Vector3  rightHand, head,leftHand, rightElbow, leftElbow;
    float d_min, d_max, h_min, h_max, Lalpha, Lbeta, Lgama, Ralpha, Rbeta, Rgama;
    std_msgs::String msg_gesture;
    stringstream str_gesture;
    //person_edges(mean_person, d_min, d_max, h_min, h_max);

    rightHand = mean_person.position.at(8);
    leftHand = mean_person.position.at(5);
    head = mean_person.position.at(0);
    rightElbow = mean_person.position.at(7);
    leftElbow = mean_person.position.at(4);
    // slope(leftHand, leftElbow, Lalpha, Lbeta, Lgama);
    // slope(rightHand, rightElbow, Ralpha, Rbeta, Rgama);

    if(reachingGesture(head,rightHand))
    {
        ROS_INFO("              *** REACHING GESTURE DETECTED (RIGHT HAND) ***");
        gest = "reach right";
    }
    else if(reachingGesture(head,leftHand))
    {
        ROS_INFO("              *** REACHING GESTURE DETECTED (Left HAND) ***");
        gest = "reach left";
    }
    else
    {
        gest = "";
        slope(rightHand, rightElbow, Ralpha, Rbeta, Rgama);
        if((Ralpha < 30)&&(Rbeta > 60)&&(Rgama >60))
        {
            ROS_INFO("              *** POINTING GESTURE DETECTED (RIGHT HAND) ***");
            gest = "point right";
        }
        slope(leftHand, leftElbow, Lalpha, Lbeta, Lgama);
        if((Lalpha < 30)&&(Lbeta > 60)&&(Lgama >60))
        {
            ROS_INFO("              *** POINTING GESTURE DETECTED (LEFT HAND) ***");
            gest = "point left";
        }

    }

    if(gest.size() > 0)
    {
        if(last_gest.compare(gest) != 0)
            gest_counter = 1;
        else
            gest_counter ++;
    } else gest_counter = 0;
    str_gesture.str("");
    if(gest_counter > 10)
        str_gesture << gest;
    msg_gesture.data = str_gesture.str();
    pub_gesture.publish(msg_gesture);
    last_gest = gest;
}
/*
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

} */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GestureRec");
    ros::NodeHandle n;
    ros::Subscriber sub_skel = n.subscribe("skeleton", 100, skeleton_cb);
    pub_gesture = n.advertise<std_msgs::String>("gesture", 100);
    last_skel = ros::Time(0);
    no_gesture.data = "";
    gest_counter = 0;
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        if ((ros::Time::now()-last_skel).toSec() > 2.0)
        {
            pub_gesture.publish(no_gesture);
            gest_counter = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
