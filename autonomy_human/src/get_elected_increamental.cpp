//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "autonomy_human/human.h"
#include "ros/time.h"
#include "autonomy_human/election.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
#define ELECTION_TIMEOUT 0.5
#define FACESCORE_TIMEOUT 0.5
#define SPEECH_TIMEOUT 200

struct Talker
{
    std_msgs::String speech;
    ros::Time ts;
}talker;

int32_t number_robot;
vector<string> sorted_namespaces;
ros::Time election_time;
autonomy_human::human face_info;
bool check_election;
bool restart_election;
std_msgs::Float32 average_fs;

// ************** Debugging Visualization Parameters & Functions
bool show_viz = true;
#define FPS_BUF_SIZE 10
float fps_ts[FPS_BUF_SIZE];
string lw_window = "Seletion Status";
int lw_width = 400;
int lw_height = 400;
cv::Mat led_vis;
string led_color;

void clearWindow()
{
    led_vis = Mat::zeros(lw_height,lw_width,CV_8UC3);
    led_color = "black";
}
void showFaceScore()
{
    clearWindow();
    char buff[25];
    sprintf(buff,"%6.1f FaceSore",average_fs.data);
    string str = buff;
    putText(led_vis,str,Point(25,25),CV_FONT_HERSHEY_PLAIN,2,CV_RGB(255,255,255));
}
void visualizeLed()
{
    imshow(lw_window,led_vis);
    waitKey(1);
}
void greenLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,255,0),-1);
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,255,255),10);
    led_color = "green";
}
void redLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,0,0),-1);
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,255,255),10);
    led_color = "red";
}
void blueLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,0,255),-1);
    led_color = "blue";
}
void happyLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,255,0),-1);
    putText(led_vis,"Selected",Point(80,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));
}
void sadLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,0,0),-1);
    putText(led_vis,"Rejected",Point(90,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));
}
void turnOffLED()
{
    clearWindow();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,0,0),-1);
    led_color = "black";
    putText(led_vis,"No Face!",Point(80,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));

}

// ~~~~~~~~~~~~~~~ Debugging Visualization Parameters & Functions

// ****************Circular queue for averaging over face scores

const int MAX_AVERAGE_FACESCORE = 5;
class cqueue
{
public:
    int arr[MAX_AVERAGE_FACESCORE],front;
    bool full_arr;
    cqueue()
    {
        front = -1;
        full_arr = false;
    }
    void insert(int );
    float_t average(int[] );
    void empty();
};

void cqueue::empty()
{
    front = -1;
    full_arr = false;
}

void cqueue::insert(int val)
{
    if(front == -1)
    {
        front = 0;
        arr[front] = val;
    }
    else if(front == MAX_AVERAGE_FACESCORE -1)
    {
        front = 0;
        arr[front] = val;
        full_arr = true;
    }
    else
    {
        front++;
        arr[front] = val;
    }
}

float_t cqueue::average(int arr[])
{
    float_t aver;
    int sum = 0;
    for(int i = 0; i < MAX_AVERAGE_FACESCORE;i++)
    {
        sum = sum + arr[i];
    }
    aver = sum/MAX_AVERAGE_FACESCORE;
    return aver;
}
cqueue cq;

// ~~~~~~~~~~~~~~~~~Circular queue for averaging over face scores

void electionResultsCallback(const autonomy_human::election& msg) // Retreive Infromation from "election" topic.
{

        number_robot = msg.num_robot;
        sorted_namespaces = msg.namespaces;
        election_time = msg.header.stamp;
        ROS_INFO("The User wants: [ %d ]  robots",number_robot);
        if(msg.is_valid)
        {
            restart_election = true;
            check_election = true;
        }
        else if(!msg.is_valid) restart_election = false;
}

void humanCallback(const autonomy_human::human& msg) // Get recent face score and put it in the faceScore to get average on FS
{
    face_info.faceScore = msg.faceScore;
    face_info.header.stamp = ros::Time::now();
    cq.insert(msg.faceScore);
    if(check_election)
    {
        if(cq.full_arr)
        {
            average_fs.data = cq.average(cq.arr);
        }
    }
    else
    {
        average_fs.data = -1;
    }
    ROS_INFO("My Average Face score: %f",average_fs.data);
    ROS_INFO("My Face score: %d",face_info.faceScore);
}

void speechCallback (const std_msgs::String& msg) // Speech commands
{
    talker.speech.data = msg.data;
    talker.ts = ros::Time::now();
    ROS_INFO("*********************  I heard: %s",talker.speech.data.c_str());
}

void isElected(vector<string> sorted_ns, string& myname, bool& iselected, unsigned int& myposition)
{
    iselected = false;
    if (!sorted_ns.empty() && (myname == sorted_ns.at(0)) && check_election) iselected = true;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_elected");
    ros::NodeHandle nh;
    ros::Time _now = ros::Time::now();

    namedWindow(lw_window); //visualization
    clearWindow(); //visualization
    turnOffLED();
    check_election = true;
    restart_election = true;

    string my_name = "Shokoofeh"; // default name
    unsigned int my_position = 0;
    bool is_elected = false;
    ros::param::get("~myName", my_name); // myName gets the robot name from user.
    face_info.faceScore = 0; // default face score

    ros::Publisher aver_faceScore_pub = nh.advertise<std_msgs::Float32>("average_facescore",10);
    ros::Subscriber election_sub=nh.subscribe("/election",10,electionResultsCallback);
    ros::Subscriber human_sub=nh.subscribe("human",10,humanCallback);
    ros::Subscriber speech_sub=nh.subscribe("/recognizer/output",10,speechCallback);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ROS_INFO("My name is: [ %s ]",my_name.c_str());
        ros::Duration last_election = ros::Time::now() - election_time;
        ros::Duration last_face_info = ros::Time::now() - face_info.header.stamp;
        ros::Duration talker_last_ts = ros::Time::now() - talker.ts;
        if(show_viz) visualizeLed();

        if(check_election){

            // Can it see a face?
                if((last_face_info.toSec() > FACESCORE_TIMEOUT) || (face_info.faceScore == 0)) // No - It can not see any face
               {
                    face_info.faceScore = 0;
                    turnOffLED();
                    if (last_face_info.toSec() > FACESCORE_TIMEOUT) ROS_ERROR("Last seen face %f seconds ago",last_face_info.toSec()); // Get the expired face score
                }
                else // Yes - It can see a face!
                {
                    // Does it get the recent election result?
                    if (last_election.toSec() > ELECTION_TIMEOUT)  // No - Either the election node is off or the result is too old
                    {
                        ROS_ERROR("Last election happened %f seconds ago",last_election.toSec());
                        is_elected = false;
                        blueLED();
                    }
                    else // Yes - It can get proper election result --> Everything is ready for decision on election.
                    {
                        // check if the election is reset or not
                        if ((talker_last_ts.toSec() < SPEECH_TIMEOUT) &&     ((strcmp(talker.speech.data.c_str(),"start") == 0) ||
                                                                                (strcmp(talker.speech.data.c_str(),"start election") == 0) ||
                                                                                (strcmp(talker.speech.data.c_str(),"restart election") == 0) ||
                                                                                (strcmp(talker.speech.data.c_str(),"restart") == 0) ||
                                                                                (strcmp(talker.speech.data.c_str(),"again") == 0))
                            )// if yes --> reset the election
                        {
                            is_elected = false;
                            restart_election = true;
                            check_election = true;
                        }

                         else   // if not --> continue the election process
                        {
                            //check to see if it is already elected or not
                            isElected(sorted_namespaces,my_name,is_elected,my_position);
                            if(is_elected) //if already elected
                            {
                                check_election = false;
                                happyLED();
                            }
                            else //if not elected
                            {
                                redLED();
                            }
                        }
                    }
                }
        }


        ROS_INFO("Am I elected? [ %d ]",is_elected);
        aver_faceScore_pub.publish(average_fs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


