#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"
#include "autonomy_human/human.h"
#include "ros/time.h"
#include "autonomy_human/SortedNamespaces.h"

#include <sstream>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

uint32_t number_robot;
vector<string> sorted_namespaces;
ros::Time election_time;
autonomy_human::human face_info;

// ************** Debugging Visualization Parameters & Functions
bool show_viz = true;
#define FPS_BUF_SIZE 10
float fps_ts[FPS_BUF_SIZE];
string lw_window = "Seletion Status";
int lw_width = 400;
int lw_height = 400;
cv::Mat led_vis;

void clearWindow()
{
    led_vis = Mat::zeros(lw_height,lw_width,CV_8UC3);
    char buff[25];
    sprintf(buff,"%d FaceSore", face_info.faceScore);
    string str = buff;
    putText(led_vis,str,Point(25,25),CV_FONT_HERSHEY_PLAIN,1,CV_RGB(255,255,255));
}

void visualizeLed()
{
    imshow(lw_window,led_vis);
    waitKey(1);
}

void greenLED()
{
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,255,0),-1);
}

void redLED()
{
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,0,0),-1);
}


// ~~~~~~~~~~~~~~~ Debugging Visualization Parameters & Functions


void electionResultsCallback(const autonomy_human::SortedNamespaces& msg)
{
    //clearWindow();
    number_robot = msg.num_robot;
    sorted_namespaces = msg.namespaces;
    election_time = msg.header.stamp;
    ROS_INFO("The User wants: [ %d ]  robots",number_robot);
    ROS_INFO("The first user is : [ %s ]  ",sorted_namespaces.at(0).c_str());
}

void humanCallback(const autonomy_human::human& msg)
{
    face_info.faceScore = msg.faceScore;
    face_info.header = msg.header;
}

void isElected(vector<string> sorted_ns, string& myname, bool& iselected, unsigned int& myposition)
{
    if (!sorted_ns.empty())
    {
        for(vector<string>::size_type i=0; i!=sorted_ns.size(); i++)
        {
            if(myname == sorted_ns.at(i))
            {
                myposition = i + 1;
                if(myposition <= number_robot) iselected = true;
                else iselected = false;
            }
        }
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_elected");
    ros::NodeHandle nh;
    ros::Time _now = ros::Time::now();


    namedWindow(lw_window); //visualization
    clearWindow(); //visualization

    string my_name = "Shokoofeh";
    unsigned int my_position = 0;
    bool is_elected = false;
    ros::param::get("~myName", my_name); // numRobot gets the number of robots to be selected from user.
    face_info.faceScore = 0;

    ros::Publisher elected_pub;
    elected_pub = nh.advertise<std_msgs::Bool>("IsElected",10);

    //ros::Publisher elected_pub=nh.advertise<std_msgs::bool>("IsElected",10);
    ros::Subscriber election_sub=nh.subscribe("/election",10,electionResultsCallback);
    ros::Subscriber human_sub=nh.subscribe("human",10,humanCallback);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ROS_INFO("My name is: [ %s ]",my_name.c_str());
        ros::Duration last_election = ros::Time::now() - election_time;
        ros::Duration last_face_info = ros::Time::now() - face_info.header.stamp;
        ROS_INFO("Last election happened %f seconds ago",last_election.toSec());
        std_msgs::Bool is_elected_msg;

        if(show_viz) visualizeLed();

        if(last_face_info.toSec() > 0.5 ) face_info.faceScore = 0;

        if (last_election.toSec() < 0.5)
        {
            isElected(sorted_namespaces,my_name,is_elected,my_position);
        }
        else
        {
            is_elected = false;
            my_position = 0;
        }
        if(is_elected)
        {
            clearWindow();
            greenLED();
        }
        else
        {
            clearWindow();
            redLED();
        }
        ROS_INFO("My position is: [ %d ]",my_position);
        ROS_INFO("Am I elected? [ %d ]",is_elected);
        ROS_INFO("My Face Score is: [%d]", face_info.faceScore);
        is_elected_msg.data = is_elected;
        elected_pub.publish(is_elected_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


