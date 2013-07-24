// LAST UPDATE June 29 2012
#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "sound_play/sound_play.h"
#include "sound_play/SoundRequest.h"
#include "pi_tracker/Skeleton.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#define FPS_BUF_SIZE 10

bool show_viz = true;
float fps;
class CPolarCord
{
public:
    float r;
    float th;
    CPolarCord(float _r, float _th);
    void fromCart(float x, float y);
    void toCart(float &x, float &y);
};

CPolarCord::CPolarCord(float _r, float _th)
{
    this->r = _r;
    this->th = _th;
}
    
void CPolarCord::fromCart(float x, float y)
{
    this->r = sqrt( (x*x) + (y*y) );
    this->th = atan2(y, x);
}

void CPolarCord::toCart(float &x, float &y)
{
    x = r * cos(th);
    y = r * sin(th);
}

const std::string lw_window = "laser";
cv::Mat laser_vis;
const int lw_width = 900;
const int lw_height = 500;
const int lw_mm_width = 18000; //mm
const int lw_mm_height = 9000; //mm

void visualizeLaser()
{
    imshow (lw_window, laser_vis);
    waitKey(1);
}

void clearVisWindow()
{
    laser_vis = Mat::zeros(lw_height, lw_width, CV_8UC3);
    char buff[25];
    sprintf(buff, "%4.2f fps", fps);
    std::string str = buff;
    putText(laser_vis, str, Point(20,20), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255,0,0));
}

/**
 * 
 * @param r     : mm
 * @param th    : degree (0 standard) 
 * @param color 
 * @param size  : in px
 */
void insertPoint(float r, float th, const Scalar& color, int rad = 2)
{
    th = (th / 180.0) * 3.141596;
    CPolarCord pr(r, th);
    float x_mm, y_mm;
    pr.toCart(x_mm, y_mm);
    float x_px = (x_mm / lw_mm_width) * lw_width;
    float y_px = (y_mm / lw_mm_height) * lw_height;
    
    float row = -y_px + lw_height;
    float col = x_px + (lw_width / 2.0);
    
    circle(laser_vis, Point(col, row), rad, color);
}

#define LASER_DATA 180
#define DISTANCE 2.75
#define ROBOT_HEAD_ANGLE 90.0
using namespace std;
geometry_msgs::Twist cmd_vel;
sound_play::SoundRequest sound_stop;
sound_play::SoundRequest sound_aha;
sound_play::SoundRequest sound_findLeg;
sound_play::SoundRequest sound_lostLeg;

int IsLegCounter = 0;
int IsNotLegCounter = 0;
int exUser = 0;
int go_cb = 0;
int skel_cb_counter = 0;
int say_aha = 0;
int ls_count = 0;
float pre_legAng = 0.0;
float noise_reduction_filter = 8.0;
vector<float> temp0, temp1, temp2;
ros::Time lastSkelTime;
std::string play_req, play_findLeg, play_lostLeg, play_aha;
ros::Publisher robotsound_pub_;
pi_tracker::Skeleton Persons;

int legTracker(vector<float>,vector<float>, vector<int>);
void nearestPoint(vector<float>,int,int,int,int, float &, int &);
void legSelection(const vector<float>, const int, vector<float>, vector<float>, vector<int>);

void legsInLaserData(vector<float> ranges, int &numLegs, vector<float> &legDist,vector<float> &legAng, vector<int> &legNearestP ){
    float diffLaserData[LASER_DATA];
    double threshold = 0.1;
    float filter[LASER_DATA];
    unsigned int i,laserIndex ,firstNeg, firstPos, secondNeg, secondPos;
    numLegs = 0;
    const int legWidth = 15;
    float Ang2Leg, preLegPos, nearestDist;
    int nearestP;
    for(int i= 0; i<180; i++){
        if(ranges.at(i) >= 8.0) ranges.at(i) = 8.0;
    }
    ROS_INFO("***************************************************");
    ROS_INFO("I got the laser data");
    ROS_INFO("cmd_vel_X: %6.2f",cmd_vel.linear.x );
    ROS_INFO("cmd_vel_Z: %6.2f",cmd_vel.angular.z);
    
//    ROS_INFO("Max LASER RANGE: %f", *max_element(ranges.begin(),ranges.end()));
    // Initialize filter array with 1
    for (i = 0; i < LASER_DATA; i++){
        filter[i] = 1.00;
        diffLaserData[i] = 1.00;
    }
    
     /* Passing Laser Data from a filter: if the difference between two consecutive
      Laser Data will be more than the threshold, this quantity will be saved in
      * filter array, if not, the filter will be zero for that difference.
      * */
    for (laserIndex = 0; laserIndex < (ranges.size()-1); laserIndex++){
        diffLaserData[laserIndex] = ranges.at(laserIndex+1) - ranges.at(laserIndex);
        if ((diffLaserData[laserIndex] < threshold) && ((diffLaserData[laserIndex] > -threshold) )){
            filter[laserIndex] = 0;
        } else {
            filter[laserIndex] = diffLaserData[laserIndex];
        }
    }

/*filter out some noises*/
    for(int k = 0; k < LASER_DATA-1; k++){
        if((filter[k]-filter[k+1] > noise_reduction_filter) && (filter[k]-filter[k+1] < -noise_reduction_filter)){
            filter[k+1] = 0.0;
        }   
    }
//    
//    for(int i = 20; i < 65; i++){
//        ROS_INFO("Filter[%d]= %6.4f     diffLaser[%d]= %6.4f    Laser[%d]= %6.4f",i,filter[i],i,diffLaserData[i],i,ranges.at(i));
//        
//    }
    
//    for (int i = 0; i < LASER_DATA; i++)
//    {
//        insertPoint((ranges[i] + filter[i]) * 1000.0, (float) i, CV_RGB(0,255,0));
//    }
/* Searching for the legs */    
firstNeg = 0;
while(firstNeg < (ranges.size()-1)){
    if (filter[firstNeg] < 0.0){
        firstPos = firstNeg+1;
        while((firstPos < (firstNeg+legWidth))&& (firstPos < ranges.size())){
            if (filter[firstPos] > 0.0){
                secondNeg = firstPos+1; 
                while((secondNeg < (firstPos+legWidth))&& (secondNeg < ranges.size())){
                     if (filter[secondNeg] < 0.0){
                         secondPos = secondNeg+1;
                         while((secondPos < (secondNeg+legWidth)) && (secondPos < ranges.size())){
                             if(filter[secondPos] > 0.0){ 
                                 
                                Ang2Leg = (firstPos + secondNeg)/2;
                                nearestPoint(ranges,firstNeg,firstPos,secondNeg,secondPos,nearestDist, nearestP);
                                if(numLegs != 0){
                                    if(Ang2Leg != preLegPos || numLegs == 1){
                                        legDist.push_back(nearestDist);
                                        legAng.push_back(Ang2Leg);
                                        legNearestP.push_back(nearestP);
                                        preLegPos = Ang2Leg;
                                        numLegs++;
                                    }
                                }else {
                                    legDist.push_back(nearestDist);
                                    legAng.push_back(Ang2Leg);
                                    legNearestP.push_back(nearestP);
                                    preLegPos = Ang2Leg;
                                    numLegs =1;
                                }
                                firstNeg = secondPos;
                                secondNeg =  ranges.size();
                                firstPos = ranges.size();
                                break;
                             }//end of if(filter[secondPos] > 0)
                             ++secondPos;
                         }// end of while on secondPos
                     } // end of  if (filter[secondNeg] < 0)
                     ++secondNeg;
                }//end of while on secondNeg
            }//end of if (filter[firstPos] > 0)
            ++firstPos;
        } // end while on firstPos
    } // end of if (filter[firstNeg] < 0)
    ++firstNeg;
} //end while on firstNeg
} //end of legsInLaserData function

void laserCallback(const sensor_msgs::LaserScan &msg){
    clearVisWindow();
    int numLegs, ls_d3;
    vector<float> ranges, legDist, legAng, temp;
    vector<int> legNearestP;
//    msg.get_ranges_vec(ranges);
    ls_count++;
    ls_d3 = ls_count - ((ls_count / 3)*3);
    ROS_INFO("ls_counter: %d", ls_count);
    if(ls_count == 1 ){
        msg.get_ranges_vec(temp1);
        temp0 = temp1;
        temp2 = temp1;
    } else if(ls_count == 2){
        msg.get_ranges_vec(temp2);
        temp0 = temp2;
    } else {
        if(ls_d3 == 0){
            temp2 = temp1;
            temp1 = temp0;
            msg.get_ranges_vec(temp0);
        } else if(ls_d3 == 1){
            temp0 = temp2;
            temp2 = temp1;
            msg.get_ranges_vec(temp1);
        } else if(ls_d3 == 2){
            temp1 = temp0;
            temp0 = temp2;
            msg.get_ranges_vec(temp2);
        }
    }
    
//    ROS_INFO("temp0: %d, temp1: %d, temp2: %d", temp0.size(),temp1.size(),temp2.size());

    for (int i = 0; i < LASER_DATA; i++){
        ranges.push_back((temp0.at(i) + temp1.at(i) + temp2.at(i)) / 3.0);
    }

    legsInLaserData(ranges, numLegs, legDist, legAng, legNearestP);
    for (int i = 0; i < numLegs; i++){
        ROS_INFO("leg # %d at %6.1f in %6.4f distance", i+1, legAng.at(i), legDist.at(i));
//        IsNotLegCounter = 0;
    }
    
    for (int i = 0; i < ranges.size(); i++)
    {
        insertPoint(ranges.at(i) * 1000.0, (float) i, CV_RGB(255,255,255));
    }
    
    for (int i = 1; i < 6; i++){
            for (int j = 1; j < 9; j++){
                insertPoint(j * 1000.0, i * 30.0, CV_RGB(255,0,0));
            }
     }
    
//    for(int i = 0; i < 180; i++){
//        insertPoint(8000.0, (float) i, CV_RGB(255,0,0));
//    }
    
    legSelection(ranges,numLegs,legDist, legAng, legNearestP);
} // End of laserCallback

void nearestPoint(vector<float> ranges,int a,int b,int c,int d, float & nearestDist, int & nearestP){
    int n;
     n = a;
     for(int i = a; i < d+1; i++){
         if (ranges.at(i) <= ranges.at(n)) n = i;
     }
     nearestDist = ranges.at(n);
     nearestP = n;
} // End of nearestPoint Function

void legSelection(const vector<float> ranges, const int numLeg, vector<float> legDist, vector<float> legAng, vector<int> legNearestP){ 

    float IsLegAngle = 10.0;
    int legCounter = 1;
    if (numLeg > 0){
        
        if(numLeg > 1){ // Select the most ahead leg and put it at the begining of array.
            for(int legIndex = 0; legIndex < numLeg-1; legIndex++){
                if(fabs(legAng.at(legIndex+1)-ROBOT_HEAD_ANGLE) < fabs(legAng.at(0)-ROBOT_HEAD_ANGLE)){
                    legAng.at(0) = legAng.at(legIndex+1);
                    legDist.at(0) = legDist.at(legIndex+1);
                    legNearestP.at(0) = legNearestP.at(legIndex+1);
                }
            }
        } 
        
       if(IsLegCounter == 0 || (fabs(legAng.at(0)-pre_legAng) < IsLegAngle)) IsLegCounter++ ;
       else if((fabs(legAng.at(0)-pre_legAng) > 2*IsLegAngle)) IsNotLegCounter++;
       
       pre_legAng = legAng.at(0);
       
       if(IsLegCounter > legCounter){
           legTracker(legDist,legAng,legNearestP);
           IsNotLegCounter = 0;
           ROS_INFO("I am following the leg. Counter: %d", IsLegCounter);
           if (IsLegCounter == legCounter + 1){
               sound_findLeg.sound = sound_play::SoundRequest::PLAY_FILE; 
               sound_findLeg.command = sound_play::SoundRequest::PLAY_ONCE;
               sound_findLeg.arg = play_findLeg;
              // robotsound_pub_.publish(sound_findLeg);
           }
       }
   } else{
       IsNotLegCounter++;
       
       if(IsNotLegCounter > legCounter){
           if (IsNotLegCounter == legCounter + 1){
               sound_lostLeg.sound = sound_play::SoundRequest::PLAY_FILE; 
               sound_lostLeg.command = sound_play::SoundRequest::PLAY_ONCE;
               sound_lostLeg.arg = play_lostLeg;
               //robotsound_pub_.publish(sound_lostLeg);
           }
           cmd_vel.linear.x = 0.0;
           cmd_vel.angular.z = 0.0;
           IsLegCounter = 0;
           ROS_INFO("ERROR! There is no body in the robot view! Counter: %d", IsNotLegCounter);
       }
   } 
} // End of legSelection function

void skeleton_cb(const pi_tracker::Skeleton &msg)
{

    Persons = msg;
    float eps = 0.0001;
    
    if (skel_cb_counter > 3){ // The /skeleton freq. is very high, So I drop 2 out of 3.
        skel_cb_counter = 0;
        int valid_counter = Persons.name.size();
        
        for (int i = 0; i < Persons.name.size(); ++i ){
            
               if(fabs(Persons.position.at(i).z / 1000.0) < eps){
                   Persons.name.at(i) = "invalid";
                   valid_counter = valid_counter -1;
               } 
        }
        
        ROS_INFO("[Kinect] I am looking at %d persons.", valid_counter);
        
        for (int i = 0; i < Persons.name.size(); ++i ){
            if(Persons.name.at(i) != "invalid")
            ROS_INFO("Detected Object at: %6.4f Degree & at: %6.4f Distance" ,Persons.position.at(i).x / 1000.0,Persons.position.at(i).z / 1000.0);
        }
    }
    skel_cb_counter ++;
//    lastSkelTime = ros::Time::now();
//    int isUser;
//    isUser = msg.user_id;
//    go_cb = 1;
//    if(isUser != exUser){
//            sound_aha.sound = sound_play::SoundRequest::PLAY_FILE; 
//            sound_aha.command = sound_play::SoundRequest::PLAY_ONCE;
//            sound_aha.arg = play_aha;
//            robotsound_pub_.publish(sound_aha);
//            exUser = isUser;
//    }
    
} // End of skeleton_cb

int legTracker(vector<float> Distance, vector<float> Angle, vector<int> P){
    float error_angle;
    float K1 = 6.0; //threshold
    error_angle = Angle.at(0) - ROBOT_HEAD_ANGLE;
    cmd_vel.angular.z = ((1/(1+exp(-error_angle/12.0))) - 0.5); 
   
    if (fabs(error_angle) < K1){
        cmd_vel.linear.x = 0.35 * (1/(1+exp((Distance.at(0)-2.0)/15.0))); 
        
        
        if  ((Distance.at(0) < DISTANCE) || (P.at(0) < DISTANCE) || (fabs(error_angle) > 20.0)){
            ROS_INFO("Distance: %6.4f", Distance.at(0));
//            ROS_INFO("Nearest Point: %6.2f", P.at(0));
            
            for(int i=0; i<Persons.position.size();i++){
                if (fabs(Persons.position.at(i).z/1000.0 - Distance.at(0)) < 0.3 && say_aha == 0){
                    sound_aha.sound = sound_play::SoundRequest::PLAY_FILE; 
                    sound_aha.command = sound_play::SoundRequest::PLAY_ONCE;
                    sound_aha.arg = play_aha;
                   // robotsound_pub_.publish(sound_aha);
                    ROS_INFO("Right Target !");
                    say_aha = 1;
                }
            }
            
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_INFO("I find my target");

//            if(exUser == 0 && cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0){
//               if (go_cb == 0){
//		   ROS_INFO("I am saying raise");
//                   sound_stop.sound = sound_play::SoundRequest::SAY; 
//                   sound_stop.command = sound_play::SoundRequest::PLAY_ONCE;
//                   sound_stop.arg = play_req;
//                   robotsound_pub_.publish(sound_stop);
//                   exUser = -1;
//               }       
//            } 
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return 0;
        }
        cmd_vel.angular.z = 0.0; // stop rotating
    }
    if(fabs(cmd_vel.angular.z) > 0.1 || fabs(cmd_vel.linear.x) > 0.1){
        say_aha = 0;
    }
    return 0;
} // End of legTracker function

// *************************** main function
int main(int argc, char **argv)
{
 ros::init(argc,argv, "findLegs");
 ros::NodeHandle n;
 ros::Rate loopRate(10);
 lastSkelTime = ros::Time::now() - ros::Duration(3600);
 
 cmd_vel.linear.y = cmd_vel.angular.z = cmd_vel.linear.x = 0.0;
 n.param<std::string>("play_req", play_req, "Please raise your hand.");
 n.param<std::string>("play_findLeg", play_findLeg, "/home/autolab/ros/ros_workspace/RobotSounds/I/IHey2.ogg");
 n.param<std::string>("play_aha", play_aha, "/home/autolab/ros/ros_workspace/RobotSounds/I/IWohoo1.ogg");
 n.param<std::string>("play_lostLeg", play_lostLeg, "/home/autolab/ros/ros_workspace/RobotSounds/I/IFailure2.ogg");
 ros::Subscriber sub = n.subscribe("scan", 100, laserCallback);
 ros::Subscriber skeleton_sub = n.subscribe("skeleton", 100, skeleton_cb);
 ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
// robotsound_pub_ = n.advertise<sound_play::SoundRequest>("robotsound",10);
 
 
 namedWindow(lw_window);
 clearVisWindow();
 
// insertPoint(0.0, 90.0, CV_RGB(255,0,0));
// insertPoint(8000.0, 90.0, CV_RGB(0,255,0));
// insertPoint(8000.0, 180.0, CV_RGB(0,0,255));
// clearVisWindow();
// insertPoint(8000.0, 0.0, CV_RGB(255,255,255));
// insertPoint(7000.0, 5.0, CV_RGB(255,0,255));
// 
 
 
 float fps_ts[FPS_BUF_SIZE];
 int counter = 0;
 ros::Time _now = ros::Time::now();
 while(ros::ok()){
     ros::Duration diffSkel = ros::Time::now() - lastSkelTime;
     if ((diffSkel.toSec() > 1.0) && (go_cb != 0)){
	go_cb = 0;
        exUser = 0;
     } 
     cmd_vel_pub_.publish(cmd_vel);
     
     if (show_viz) visualizeLaser();
     ros::spinOnce();
     fps_ts[counter] = (ros::Time::now() - _now).toNSec();
     _now = ros::Time::now();
     counter = (counter + 1) % FPS_BUF_SIZE;
     float sum = 0.0;
     for (int i = 0; i < FPS_BUF_SIZE; i++)
     {
         sum += fps_ts[i];
     }
     fps = 1e9 / (sum / ((double) FPS_BUF_SIZE));
     loopRate.sleep();
 }
 return 0;
} //End of main 
