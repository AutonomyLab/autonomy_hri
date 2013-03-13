//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "autonomy_human/human.h"
#include "ros/time.h"
#include "autonomy_human/election.h"
#include "std_msgs/Float32.h"
#include "ardrone_autonomy/LedAnim.h"
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/Navdata.h>
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
#define SPEECH_TIMEOUT 0.5
#define HOVERING_TIMEOUT 2

const unsigned int MAX_AVERAGE_ELECTION = 3;
const unsigned int MAX_AVERAGE_FACESCORE = 5;

struct Talker
{
    std_msgs::String speech;
    ros::Time ts;
}talker;

enum
{
    wait4FaceSTATE,
    wait4ElectionSTATE,
    electedSTATE,
    rejectedSTATE,
    docommandSTATE,
    flippingSTATE,
    hoveringSTATE,
}robot_state, last_state;

enum _DRONE_MODE {
    MODE_WAITINGFORDRONE,
    MODE_RESETTED,
    MODE_TAKINGOFF,
    MODE_WAITINGFORSTABLEPOSE,
    MODE_OPERATION,
    MODE_LANDING,
    MODE_ANIM,
    MODE_NUM
} droneMode;

enum _DRONE_STATES {
    STATE_UNKNOWN = 0,
    STATE_INITED,
    STATE_LANDED,
    STATE_FLYING1,
    STATE_HOVERING,
    STATE_TEST,
    STATE_TAKINGOFF,
    STATE_FLYING2,
    STATE_LANDING,
    STATE_LOOPING,
    STATE_NUM
};

enum _DRONE_COMMANDS {
    CMD_NOP,
    CMD_TAKEOFFLAND,
    CMD_RESET,
    CMD_VELOCITY,
    CMD_HOVER,
    CMD_NUM
} droneCommand;

string state_names[5] =
{
    "Looking for Face",
    "Waiting for election msg",
    "Elected",
    "Rejected",
    "Taking off"
};

int32_t number_robot;
vector<string> sorted_namespaces;
ros::Time election_time, stateTime;
ros::Duration last_election, last_face_info, talker_last_ts;
autonomy_human::human face_info;
bool check_election, restart_election, is_elected, valid_election, is_rejected;
std_msgs::Float32 average_fs;
string my_name;
unsigned int my_position, elected_counter, rejected_counter, hovering_counter;
ardrone_autonomy::LedAnim ardrone_led;
ardrone_autonomy::Navdata droneStatus;
ros::ServiceClient led_serv;
ros::ServiceClient anim_serv;
std_msgs::Empty takeoff,land,reset;
ros::Publisher takeoff_pub,land_pub, reset_pub;

void playLEDAnimation(const int _type, const int _duration, const float _freq )
{
    ardrone_autonomy::LedAnim ledAnim;
    static bool isFirst = true;
    static ros::Time lastRequest;
    static int lastDuration;

    if (isFirst)
    {
        lastRequest = ros::Time::now();
        lastDuration = -1;
        isFirst = false;
    }
    else
    {
        // Return if still executing last animation
        if ( (ros::Time::now() - lastRequest) < ros::Duration( ((double) lastDuration) - 0.25)) return;
    }


    ledAnim.request.type = _type;
    ledAnim.request.duration = _duration;
    ledAnim.request.freq = _freq;

    lastRequest = ros::Time::now();
    lastDuration = _duration;

    if (led_serv)
    {
        led_serv.call(ledAnim);
    }
    else
    {
        ROS_WARN("Led Animation Service not working!");
    }

}


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
    playLEDAnimation( 8, 1, 1.0);
}
void redLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,0,0),-1);
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,255,255),10);
    led_color = "red";
    playLEDAnimation( 7, 1, 1.0);
}
void blueLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,0,255),-1);
    led_color = "blue";
    playLEDAnimation( 3, 1, 1.0);
}
void happyLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,255,0),-1);
    putText(led_vis,"Selected",Point(80,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));
    playLEDAnimation( 8, 1, 1.0);
}
void sadLED()
{
    showFaceScore();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(255,0,0),-1);
    putText(led_vis,"Rejected",Point(90,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));
    playLEDAnimation( 7, 1, 1.0);
}
void turnOffLED()
{
    clearWindow();
    circle(led_vis,Point(lw_height/2,lw_width/2),150,CV_RGB(0,0,0),-1);
    led_color = "black";
    putText(led_vis,"No Face!",Point(80,220),CV_FONT_HERSHEY_PLAIN,3,CV_RGB(255,255,255));
    playLEDAnimation( 2, 1, 1.0);
}

// ~~~~~~~~~~~~~~~ Debugging Visualization Parameters & Functions

// ****************Circular queue for averaging over face scores


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
    for(unsigned int i = 0; i < MAX_AVERAGE_FACESCORE;i++)
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
        if(msg.is_valid) valid_election = true;
        else if(!msg.is_valid) valid_election = false;
        ROS_INFO("The User wants: [ %d ]  robots",number_robot);
}

void humanCallback(const autonomy_human::human& msg) // Get recent face score and put it in the faceScore to get average on FS
{
    face_info.faceScore = msg.faceScore;
    face_info.header.stamp = ros::Time::now();
    cq.insert(msg.faceScore);
    ROS_INFO("My Face score: %d",face_info.faceScore);
}

void speechCallback (const std_msgs::String& msg) // Speech commands
{
    talker.speech.data = msg.data;
    talker.ts = ros::Time::now();
    ROS_INFO("*********************  I heard: %s",talker.speech.data.c_str());
}

void navdataCallback(const ardrone_autonomy::NavdataConstPtr &msg)
{
    // First message indicates Drone is ready
    if (droneMode == MODE_WAITINGFORDRONE)
    {
        droneMode = MODE_RESETTED;
        if (msg->state == 0)
        {
            ROS_WARN("Resetting drone");
            reset_pub.publish(reset);
        }
    }

    droneStatus = *msg;
}



void aveFacescoreFunc(){
    if(check_election)
    {
        if(cq.full_arr)
        {
            average_fs.data = cq.average(cq.arr);
        }else
        {
             average_fs.data = 0;
        }
    } else if(!check_election && (robot_state == electedSTATE))
    {
        average_fs.data = -cq.average(cq.arr);;
    } else if(!check_election && (robot_state == rejectedSTATE))
    {
         average_fs.data = 0;
    }
    ROS_INFO("My Face score: %d",face_info.faceScore);
}

void isElected(vector<string> sorted_ns, string& myname, bool& iselected, unsigned int& myposition)
{
    iselected = false;
    if (!sorted_ns.empty() && (myname == sorted_ns.front()) && valid_election && check_election && (number_robot > 0)) {
        elected_counter += 1;
        ROS_INFO("Elected Counter:    %d", elected_counter);
        if(elected_counter > MAX_AVERAGE_ELECTION)
        {
        iselected = true;
        }
    }
}

void isRejected(vector<string> sorted_ns, string& myname, bool& isrejected)
{
    isrejected = false;
    if (!sorted_ns.empty() && (myname == sorted_ns.back()))
    {
        rejected_counter = rejected_counter + 1;
        ROS_INFO("Rejected Counter:    %d", rejected_counter);
        if(rejected_counter > MAX_AVERAGE_ELECTION)
        {
        isrejected = true;
        }
    }
}

void wait4FaceFunc()
{
    if (last_state != wait4FaceSTATE) stateTime = ros::Time::now();
    face_info.faceScore = 0;
    turnOffLED();
    last_state = wait4FaceSTATE;
}

void wait4ElectionFunc()
{
    if (last_state != wait4ElectionSTATE) stateTime = ros::Time::now();
    check_election = true;
    // Does it get the recent election result?
    if (last_election.toSec() > ELECTION_TIMEOUT)  // No - Either the election node is off or the result is too old
    {
        ROS_ERROR("Last election happened %f seconds ago",last_election.toSec());
        is_elected = false;
        blueLED();
    }
    else  // Yes - It can get proper election result --> Everything is ready for decision on election.
    {
        redLED();
        isElected(sorted_namespaces,my_name,is_elected,my_position);
        if(is_elected) //if already elected
        {
            robot_state = electedSTATE;
            check_election = false;
//            if(!is_rejected)
//            {
//                robot_state = electedSTATE;
//                check_election = false;
//            } else
//            {
//                robot_state = rejectedSTATE;
//                check_election = false;
//            }
        }
    }

    last_state = wait4ElectionSTATE;
}

void electedFunc()
{
    if (last_state != electedSTATE) stateTime = ros::Time::now();
    check_election = false;
    is_rejected = false;
    happyLED();
    // check if the election is reset or not
    if ((talker_last_ts.toSec() < SPEECH_TIMEOUT) &&     ((strcmp(talker.speech.data.c_str(),"start") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"start election") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"restart election") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"restart") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"again") == 0)))// if yes --> reset the election
    {
        is_elected = false;
        is_rejected = false;
        restart_election = true;
        rejected_counter = 0;
        check_election = true;
        robot_state = wait4FaceSTATE;
        elected_counter = 0;
        number_robot = 0;
        talker.speech.data.clear();
        cq.empty();
    } else if ((talker_last_ts.toSec() < SPEECH_TIMEOUT) &&     ((strcmp(talker.speech.data.c_str(),"not you") == 0)))
    {
        isRejected(sorted_namespaces,my_name,is_rejected);
        if (is_rejected)
        {
            talker.speech.data.clear();
            sadLED();
            //elected_counter = 0;
            robot_state = rejectedSTATE;
        }

    } else if ((talker_last_ts.toSec() < SPEECH_TIMEOUT) &&     ((strcmp(talker.speech.data.c_str(),"take off") == 0)))
    {
        ROS_ERROR("I heard somthing ~~~~~~~~~~~~~~~~~~~~~` takeoff????");
        robot_state = docommandSTATE;
        talker.speech.data.clear();
    }
    last_state = electedSTATE;
}

void rejectedFunc()
{
    if (last_state != rejectedSTATE) stateTime = ros::Time::now();
    check_election = false;
    sadLED();

    // check if the election is reset or not
    if ((talker_last_ts.toSec() < SPEECH_TIMEOUT) &&     ((strcmp(talker.speech.data.c_str(),"start") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"start election") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"restart election") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"restart") == 0) ||
                                                            (strcmp(talker.speech.data.c_str(),"again") == 0)))// if yes --> reset the election
    {
        is_elected = false;
        is_rejected = false;
        restart_election = true;
        check_election = true;
        robot_state = wait4FaceSTATE;
        rejected_counter = 0;
        elected_counter = 0;
        number_robot = 0;
        talker.speech.data.clear();
        cq.empty();
    }
    last_state = rejectedSTATE;
}

void docommandFunc()
{
    if(last_state != docommandSTATE) stateTime = ros::Time::now();
    ROS_ERROR("HELLOOOO THERE");
    if(droneStatus.state == 2) //STATE_LANDED
    {
        // take off
        takeoff_pub.publish(takeoff);

    }else if (droneStatus.state == STATE_TAKINGOFF)
    {
        // Do nothing
    }else if (droneStatus.state == STATE_HOVERING)
    {
        //increase hover_counter
        hovering_counter ++;
        if(hovering_counter > HOVERING_TIMEOUT)
        {
            //land
            robot_state = wait4FaceSTATE;
            land_pub.publish(land);
        }
    }
    last_state = docommandSTATE;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_elected_demo");
    ros::NodeHandle nh;
    ros::Time _now = ros::Time::now();
    stateTime = ros::Time::now();
    robot_state = wait4FaceSTATE;
    last_state = wait4FaceSTATE;

    namedWindow(lw_window); //visualization
    clearWindow(); //visualization
    turnOffLED();
    check_election = true;
    restart_election = true;
    elected_counter = 0;
    rejected_counter = 0;
    hovering_counter = 0;

    my_name = "Shokoofeh"; // default name
    my_position = 0;
    is_elected = false;
    is_rejected = false;
    ros::param::get("~myName", my_name); // myName gets the robot name from user.
    face_info.faceScore = 0; // default face score
    droneMode = MODE_WAITINGFORDRONE;

    ros::Publisher aver_faceScore_pub = nh.advertise<std_msgs::Float32>("average_facescore",10);
    takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 100);
    land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 100);
    reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 100);
    ros::Subscriber election_sub=nh.subscribe("/election",10,electionResultsCallback);
    ros::Subscriber human_sub=nh.subscribe("human",10,humanCallback);
    ros::Subscriber speech_sub=nh.subscribe("/recognizer/output",10,speechCallback);
    ros::Subscriber nav_sub = nh.subscribe("ardrone/navdata", 50, navdataCallback);
    led_serv = nh.serviceClient<ardrone_autonomy::LedAnim>("ardrone/setledanimation");
    anim_serv = nh.serviceClient<ardrone_autonomy::FlightAnim>("ardrone/setflightanimation");

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if(droneMode == MODE_WAITINGFORDRONE)
        {
            ROS_INFO_THROTTLE(1,"Waiting for drone");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        last_election = ros::Time::now() - election_time;
        last_face_info = ros::Time::now() - face_info.header.stamp;
        talker_last_ts = ros::Time::now() - talker.ts;
        if(show_viz) visualizeLed();
        aveFacescoreFunc();



        if(check_election){
            if((last_face_info.toSec() > FACESCORE_TIMEOUT) ) // No - It can not see any face
            {
                ROS_ERROR("Last seen face %f seconds ago",last_face_info.toSec()); // Get the expired face score
                robot_state = wait4FaceSTATE;
            }
            else // Yes - It can see a face!
            {
                 robot_state = wait4ElectionSTATE;
            }
        } else if(robot_state != docommandSTATE)
        {
            if(is_rejected) robot_state = rejectedSTATE;
            else robot_state = electedSTATE;
        }


        //********************* ROBOT STATE *********************

        if (robot_state == wait4FaceSTATE)
        {
            wait4FaceFunc();
        } else if (robot_state == wait4ElectionSTATE)
        {
            wait4ElectionFunc();
        } else if (robot_state == electedSTATE)
        {
            electedFunc();
        } else if (robot_state == rejectedSTATE)
        {
            rejectedFunc();
        }else if (robot_state == docommandSTATE)
        {
            docommandFunc();
        }



        ROS_INFO("***************************************************");
        ROS_INFO("Robot State: >>> %s <<<", state_names[robot_state].c_str());
        ROS_INFO("Am I elected? [ %d ]",is_elected);
         ROS_INFO("Am I rejected? [ %d ]",is_elected);
         //ROS_ERROR("Averege Face Score: [%f] ", average_fs.data);
        aver_faceScore_pub.publish(average_fs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



