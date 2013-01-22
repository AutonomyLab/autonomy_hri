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

using namespace std;
using std::vector;

//autonomy_human::SortedNamespaces election_results;
uint32_t number_robot;
vector<string> sorted_namespaces;
ros::Time election_time;

void electionResultsCallback(const autonomy_human::SortedNamespaces& msg)
{
    number_robot = msg.num_robot;
    sorted_namespaces = msg.namespaces;
    election_time = msg.header.stamp;
    //gonk_robot.fs = msg.faceScore;
    //gonk_robot.ts = ros::Time::now();
    ROS_INFO("The User wants: [ %d ]  robots",number_robot);
    ROS_INFO("The first user is : [ %s ]  ",sorted_namespaces.at(0).c_str());
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
    string my_name = "Shokoofeh";
    unsigned int my_position = 0;
    bool is_elected = false;
    ros::param::get("~myName", my_name); // numRobot gets the number of robots to be selected from user.

    ros::Publisher elected_pub;
    elected_pub = nh.advertise<std_msgs::Bool>("IsElected",10);

    //ros::Publisher elected_pub=nh.advertise<std_msgs::bool>("IsElected",10);
    ros::Subscriber election_sub=nh.subscribe("/election",10,electionResultsCallback);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ROS_INFO("My name is: [ %s ]",my_name.c_str());
        ros::Duration last_election = ros::Time::now() - election_time;
        ROS_INFO("Last election happened %f seconds ago",last_election.toSec());
        std_msgs::Bool is_elected_msg;

        if (last_election.toSec() < 2)
        {
            isElected(sorted_namespaces,my_name,is_elected,my_position);
        }
        else
        {
            is_elected = false;
            my_position = 0;
        }
        ROS_INFO("My position is: [ %d ]",my_position);
        ROS_INFO("Am I elected? [ %d ]",is_elected);
        is_elected_msg.data = is_elected;
        elected_pub.publish(is_elected_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


