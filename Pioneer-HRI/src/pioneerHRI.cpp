#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "sound_play/sound_play.h"
#include "sound_play/SoundRequest.h"
#include "pi_tracker/Skeleton.h"
#include "p2os_driver/SonarArray.h"
#include "p2os_driver/GripperState.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>
#include <deque>
#include <math.h>
#include "diagnostic_msgs/DiagnosticArray.h"

#define FPS_BUF_SIZE 10
#define LASER_DATA 180 
#define DISTANCE 3.00 // Meters
#define ROBOT_HEAD_ANGLE 90.0
#define WAIT_COUNTER 100
#define LASER_MAX_RANGE 8.0
#define WANDER_SPEED 0.3
#define LOOP_RATE 20
#define STATE_TIME 25
#define PI 3.14
#define LEG_WIDTH  10


using namespace cv;
using namespace std;

// **************************** VARABLES (start)

typedef struct  {
    float angle;
    float dist;
    float nearestDist;
    bool isValid;
    uint16_t start_angle;
    uint16_t end_angle;
}_leg;

bool wall = false;
bool IsTracking;
bool reachTarget = false; // True if robot is in 3 meters of legs.
bool ready2get = false;
bool IsKinectConfirm = false;
bool gripper_cmd;
bool should_exit = false;
bool giveOK = false;
bool pEStop = false;

unsigned int IsLegCounter = 0;
unsigned int last_IsLegCounter;
unsigned int IsNotLegCounter = 0;
unsigned int last_IsNotLegCounter;
unsigned int exUser = 0;
unsigned int go_cb = 0;
unsigned int skel_cb_counter = 0;
unsigned int say_aha = 0;
unsigned int ls_count = 0;
unsigned int rejectCounter = 0;
unsigned int confirmCounter = 0;
unsigned int last_confirmCounter = 0;
unsigned int last_rejectCounter = 0;
unsigned int exitIgnoring = 0;
int8_t wanderDir = 1;
int8_t TurnDir = 1;
unsigned int wait4CommandCounter = 0;
unsigned int wait4ObjectCounter = 0;
uint8_t tempNumLegs;
uint16_t start_range, end_range;

uint8_t numLegs;
uint8_t LEG_COUNTER = (LOOP_RATE)/2;
int8_t last_gripper_cmd = 0;
unsigned int exitIgnoringCounter = 0;
unsigned int legFOV;
unsigned int givingCounter = 0;
unsigned int soundRankCounter = 0;
uint8_t soundRank = 10;
float pre_legAng = 0.0;

string play_req, play_findLeg, play_ignore, play_aha, play_reachgesture, play_noGest;
string play_wander, play_reachuser, play_pointgesture, play_np, play_es, play_noJoints, play_allJoints;


vector<float> laser_ranges; //Raw Laser data
// Always hold all legs regardless of state
vector<float> tempLegsDist;
vector<float> tempLegAng;
vector<float> tempLegNP;
vector<double> sonar_ranges; //Raw Laser data
vector<float> now_range, old_range, oldest_range;
vector<float> legDist, legAng;
vector<float> legNearestP;
vector<uint16_t> legStart, legEnd, templegStart, templegEnd;
deque< vector<float> > hist_laser;

sound_play::SoundRequest sound_stop;
sound_play::SoundRequest sound_findUser;
sound_play::SoundRequest sound_reachGesture;
sound_play::SoundRequest sound_findLeg;
sound_play::SoundRequest sound_ignore;
sound_play::SoundRequest sound_noGest;
sound_play::SoundRequest sound_wander;
sound_play::SoundRequest sound_reachuser;
sound_play::SoundRequest sound_pointGesture;
sound_play::SoundRequest sound_noProblem;
sound_play::SoundRequest sound_ES;
sound_play::SoundRequest sound_allJoints;
sound_play::SoundRequest sound_noJoints;


ros::Time lastSkelTime;
ros::Time lastPersonTime;
ros::Time lastLaserTime;
ros::Time stateTime;
ros::Publisher robotsound_pub;
ros::Publisher userID_pub;

pi_tracker::Skeleton Persons;

std_msgs::String gest;
std_msgs::Int32 userID, jointConf;
geometry_msgs::Twist cmd_vel;
p2os_driver::GripperState gripper_control;
p2os_driver::GripperState gripper_state;
diagnostic_msgs::DiagnosticArray soundDiag;
vector <diagnostic_msgs::DiagnosticStatus> Status;
diagnostic_msgs::DiagnosticStatus soundStatus;

_leg targetLeg; // Target Leg for tracking
_leg ignoreLeg;

enum _state{
    wait4laserSTATE,
    wanderSTATE,
    wait4CommandSTATE,
    doubtTrackSTATE,
    ignoreSTATE,
    reachUserSTATE,
    gettingObjectSTATE,
    backOffSTATE,
    givingObjectSTATE,
};
//robot_state, last_state;

_state robot_state ;
_state last_state ;

string state_names[9] = {
    "Waiting 4 Laser Data",
    "Wandering",
    "Waiting 4 Command",
    "Doubt Tracking",
    "Ignoring",
    "Reaching User",
    "Getting Object",
    "Backing Off",
    "Giving Object"
};

// **************************** VARABLES (end)

// **************************** LASER-VISUALIZATION (start)
bool show_viz = true;
float fps;
const uint16_t LW_WIDTH = 900;
const uint16_t LW_HEIGHT = 500;
const uint16_t lw_mm_width = 18000; //mm
const uint16_t lw_mm_height = 9000; //mm
string lw_window = "laser";
cv::Mat laser_vis;
class CPolarCord {
public:
    float r;
    float th;
    CPolarCord(float _r, float _th);
    void fromCart(float x, float y);
    void toCart(float &x, float &y);
};

CPolarCord::CPolarCord(float _r, float _th) {
    this->r = _r;
    this->th = _th;
}

void CPolarCord::fromCart(float x, float y) {
    this->r = sqrt((x * x) + (y * y));
    this->th = atan2(y, x);
}

void CPolarCord::toCart(float &x, float &y) {
    x = r * cos(th);
    y = r * sin(th);
}

void visualizeLaser() {
    imshow(lw_window, laser_vis);
    waitKey(1);
}

void clearVisWindow() {
    laser_vis = Mat::zeros(LW_HEIGHT, LW_WIDTH, CV_8UC3);
    char buff[25];
    sprintf(buff, "%4.2f fps", fps);
    string str = buff;
    putText(laser_vis, str, Point(20, 20), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
}

void insertPoint(float r, float th, const Scalar& color, uint16_t rad = 2) {
    th = (th / 180.0) * 3.141596;
    CPolarCord pr(r, th);
    float x_mm, y_mm;
    pr.toCart(x_mm, y_mm);
    float x_px = (x_mm / lw_mm_width) * LW_WIDTH;
    float y_px = (y_mm / lw_mm_height) * LW_HEIGHT;

    float row = -y_px + LW_HEIGHT;
    float col = x_px + (LW_WIDTH / 2.0);

    circle(laser_vis, Point(col, row), rad, color);

}

// **************************** LASER-VISUALIZATION (end)


void nearestPoint(vector<float>, uint16_t, uint16_t, float &, uint16_t &);
void IsLeg(vector<float>, uint16_t, uint16_t, uint16_t, uint16_t, bool &);

template<typename T>
T resetCounter(T &C) {
    return(C = 0);
}

void updateKinectConfirm() {
    ROS_INFO("userNum: %lu", Persons.name.size());
    IsKinectConfirm = false;

    for (uint8_t i = 0; i < Persons.name.size(); i++) {
        if (fabs(Persons.position.at(i).z) < 1.0) {
            Persons.name.at(i) = "invalid";
        }
    }

    for (uint8_t i = 0; i < Persons.name.size(); i++) {/* TODO: FIX The parameters*/
        if ((fabs(Persons.position.at(i).z / 1000.0 - targetLeg.dist) < 1.0) && (fabs(((Persons.position.at(i).x / 10.0) + 60.0) - targetLeg.angle) < 15.0) && (Persons.name.at(i) != "invalid")) {
            ROS_INFO("There is a person at %6.0f Degree !", targetLeg.angle);
            //            ROS_INFO("Persons.name.at(i): %s", Persons.name.at(i).c_str());
            userID.data = i;
            userID_pub.publish(userID);
            IsKinectConfirm = true;
        }
    }

} /* End of updateKinectConfirm */

void legsInLaserData(vector<float> ranges, uint16_t startRange, uint16_t endRange, uint8_t &numLegs, vector<float> &legDist, vector<float> &legAng, vector<float> &legNearestP, vector<uint16_t> &legStart, vector<uint16_t> &legEnd) {

    float diffLaserData[ranges.size()];
    double threshold = 0.1;
    float filter[ranges.size()];
    uint16_t i, laserIndex, firstNeg, firstPos, secondNeg, secondPos, nearestP;
    resetCounter(numLegs);
    float Ang2Leg, NearestDist;
    float PreLegPos = -360;
    bool isLeg;

    legAng.clear();
    legDist.clear();
    legNearestP.clear();
    legStart.clear();
    legEnd.clear();

    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges.at(i) > LASER_MAX_RANGE) ranges.at(i) = LASER_MAX_RANGE;
    }

    // Initialize filter array with 1
    for (i = 0; i < ranges.size(); i++) {
        filter[i] = 1.00;
        diffLaserData[i] = 1.00;
    }

    /* Passing Laser Data from a filter: if the difference between two consecutive
     Laser Data will be more than the threshold, this quantity will be saved in
     * filter array, if not, the filter will be zero for that difference.
     * */
    for (laserIndex = 0; laserIndex < (ranges.size() - 1); laserIndex++) {
        diffLaserData[laserIndex] = ranges.at(laserIndex + 1) - ranges.at(laserIndex);
        if ((diffLaserData[laserIndex] < threshold) && ((diffLaserData[laserIndex] > -threshold))) {
            filter[laserIndex] = 0;
        } else {
            filter[laserIndex] = diffLaserData[laserIndex];
        }
    }

    /*filter out some noises*/
    for (size_t k = 0; k < ranges.size() - 1; k++) {
        if ((filter[k] - filter[k + 1] > LASER_MAX_RANGE) && (filter[k] - filter[k + 1] < -LASER_MAX_RANGE)) {
            filter[k + 1] = 0.0;
        }
    }

    /* Searching for the legs */


    firstNeg = startRange;
    while (firstNeg < endRange) {
        if (filter[firstNeg] < 0.0) {
            firstPos = firstNeg + 1;
            while ((firstPos < (firstNeg + LEG_WIDTH)) && (firstPos < endRange)) {
                if (filter[firstPos] > 0.0) {
                    secondNeg = firstPos + 1;
                    while ((secondNeg < (firstPos + LEG_WIDTH)) && (secondNeg < endRange)) {
                        if (filter[secondNeg] < 0.0) {
                            secondPos = secondNeg + 1;
                            while ((secondPos < (secondNeg + LEG_WIDTH)) && (secondPos < endRange)) {
                                if (filter[secondPos] > 0.0) {

                                    IsLeg(ranges, firstNeg, firstPos, secondNeg, secondPos, isLeg);
                                    if (!isLeg) break;
                                    Ang2Leg = (firstPos + secondNeg) / 2;
                                    nearestPoint(ranges, firstNeg, secondPos, NearestDist, nearestP);

                                    if (numLegs != 0) {
                                        if (Ang2Leg != PreLegPos || numLegs == 1) {
                                            legDist.push_back(NearestDist);
                                            legAng.push_back(Ang2Leg);
                                            legNearestP.push_back(nearestP);
                                            legStart.push_back(firstNeg);
                                            legEnd.push_back(secondPos);
                                            PreLegPos = Ang2Leg;
                                            numLegs++;
                                        }
                                    } else {
                                        legDist.push_back(NearestDist);
                                        legAng.push_back(Ang2Leg);
                                        legNearestP.push_back(nearestP);
                                        legStart.push_back(firstNeg);
                                        legEnd.push_back(secondPos);
                                        PreLegPos = Ang2Leg;
                                        numLegs = 1;
                                    }
                                    firstNeg = secondPos;
                                    secondNeg = endRange;
                                    firstPos = endRange;
                                    break;
                                }/*end of if(filter[secondPos] > 0)*/
                                ++secondPos;
                            }/* end of while on secondPos */
                        } /* end of  if (filter[secondNeg] < 0) */
                        ++secondNeg;
                    }/*end of while on secondNeg */
                }/*end of if (filter[firstPos] > 0) */
                ++firstPos;
            } /* end while on firstPos */
        } /* end of if (filter[firstNeg] < 0) */
        ++firstNeg;
    } /* end while on firstNeg */

} /* end of legsInLaserData function */

void nearestPoint(vector<float> ranges, uint16_t a, uint16_t d, float & nearestDist, uint16_t & nearestP) {
    uint16_t n = a;
    for (uint16_t i = a; i < d + 1; i++)
        if (ranges.at(i) <= ranges.at(n))
            n = i;
    nearestDist = ranges.at(n);
    nearestP = n;
} // End of nearestPoint Function

void IsLeg(vector<float> ranges, uint16_t a, uint16_t b, uint16_t c, uint16_t d, bool & isLeg) {

    float D1, D2;
    uint16_t P1, P2;
    nearestPoint(ranges, a, b, D1, P1);
    nearestPoint(ranges, c, d, D2, P2);
    if (fabs(D1 - D2) > 1.00) {
        isLeg = false;
    } else {
        isLeg = true;
    }
} // End of IsLeg Function 

bool isWall() {

    double dis;
    dis = (0 * sonar_ranges[10]) + (0.5 * sonar_ranges[11]) + (0.5 * sonar_ranges[12]) + (0 * sonar_ranges[13]);
    if (dis < 0.75) {
        return true;
    } else {
        return false;
    }
}

void Wander(int direction) {
    cmd_vel.linear.x = 0.0;
    if (direction == 0) { // Rotate Clockwise (to right)
        cmd_vel.angular.z = -WANDER_SPEED;
    } else {              // Rotate Counterclockwise (to left)
        cmd_vel.angular.z = WANDER_SPEED;
    }
}

int followLeg() {
    float error_angle;
    float error_distance;
    float K1 = 5.0; //Angle Threshold
    error_angle = targetLeg.angle - ROBOT_HEAD_ANGLE;
    error_distance = targetLeg.dist - DISTANCE;

    cmd_vel.angular.z = ((1 / (1 + exp(-error_angle / 12.0))) - 0.5);

    if (fabs(error_angle) < K1) {

        cmd_vel.angular.z = 0.0; // stop rotating

        if (targetLeg.dist < DISTANCE) {
            reachTarget = true;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return 0;
        } else {
            cmd_vel.linear.x = 0.75 * ((1 / (1 + exp(-error_distance * 2))) - 0.4);
            reachTarget = false;
        }
    }
    return 0;
} // End of followLeg function

void getObject() {


} // End of getObject() Function

int plotAllLegs() {
    if (laser_ranges.empty()) return 0; // temporary! TODO: FIX ME
    legsInLaserData(laser_ranges, 0, LASER_DATA - 1, tempNumLegs, tempLegsDist, tempLegAng, tempLegNP, templegStart, templegEnd);
    /* Plot all legs in laser FOV in grey */
    for (int i = 0; i < tempNumLegs; i++) {
        insertPoint(tempLegsDist.at(i)* 1000.0, tempLegAng.at(i), CV_RGB(127, 127, 127), 20); // GREY
    }
    return 0;
}

void legSelection() {
    float IsLegAngle = 10.0;
    bool inKinectFOV;
    float minAng;

    if (numLegs > 0) {

        if (numLegs == 1) {
            targetLeg.isValid = true;
            //            targetLeg.angle = legAng.at(0);
            targetLeg.dist = legDist.at(0);
            targetLeg.nearestDist = legNearestP.at(0);
            targetLeg.start_angle = legStart.at(0);
            targetLeg.end_angle = legEnd.at(0);
            targetLeg.angle = ((targetLeg.start_angle + targetLeg.end_angle) / 2);
        } else { /*if (numLegs > 1) { // Select one pair */
            inKinectFOV = false;
            for (int legIndex = 0; legIndex < numLegs; legIndex++) { // kinectFOV
                if ((legAng.at(legIndex) < 120.0 && legAng.at(legIndex) > 60.0) && (legDist.at(legIndex) < 6.0 && legDist.at(legIndex) > 1.5)) {
                    targetLeg.isValid = true;
                    //                    targetLeg.angle = legAng.at(legIndex);
                    targetLeg.dist = legDist.at(legIndex);
                    targetLeg.nearestDist = legNearestP.at(legIndex);
                    targetLeg.start_angle = legStart.at(legIndex);
                    targetLeg.end_angle = legEnd.at(legIndex);
                    targetLeg.angle = ((targetLeg.start_angle + targetLeg.end_angle) / 2);
                    inKinectFOV = true;
                }
            }
            /* Not in kinectFOV - Pick the most ahead leg and put it at the begining of array. */
            if (!inKinectFOV) {
                uint8_t minIndex = 0;
                minAng = fabs(legAng.at(0) - ROBOT_HEAD_ANGLE);
                for (uint8_t legIndex = 1; legIndex < numLegs; legIndex++) {
                    if (fabs(legAng.at(legIndex) - ROBOT_HEAD_ANGLE) < minAng) {
                        minAng = fabs(legAng.at(legIndex) - ROBOT_HEAD_ANGLE);
                        minIndex = legIndex;
                    }
                }
                targetLeg.isValid = true;
                //                targetLeg.angle = legAng.at(minIndex);
                targetLeg.dist = legDist.at(minIndex);
                targetLeg.nearestDist = legNearestP.at(minIndex);
                targetLeg.start_angle = legStart.at(minIndex);
                targetLeg.end_angle = legEnd.at(minIndex);
                targetLeg.angle = ((targetLeg.start_angle + targetLeg.end_angle) / 2);
            }
        }
        if (IsLegCounter == 0 || (fabs(targetLeg.angle - pre_legAng) < IsLegAngle)) {
            IsLegCounter++;
        }
        pre_legAng = targetLeg.angle;
        insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(0, 0, 255), 25); //BLUE
    } else { // There is nobody there!
        IsNotLegCounter++;
        /* 2
        if (IsNotLegCounter > LegCounter) {
            targetLeg.isValid = false;
//            robot_state = wanderSTATE;
            rejectCounter = 0;
            ROS_INFO("Temp 4");
            confirmCounter = 0;
            if (IsNotLegCounter < 10) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            } else {
                Wander(0);
            }

            IsLegCounter = 0;
            ROS_INFO("ERROR! There is no body in the robot view! Counter: %d", IsNotLegCounter);
            IsTracking = false;
        } 
      2  */
    }
}

void exitIgnoringFunc() {
    robot_state = ignoreSTATE;
    if (numLegs > 0) { /* If in Ignoring state there were a leg change the state to dounbtracking */
        exitIgnoring++;
        if (exitIgnoring > LOOP_RATE) {
            if (TurnDir == 0) {
                targetLeg.isValid = true;
                //                targetLeg.angle = legAng.back();
                targetLeg.dist = legDist.back();
                targetLeg.nearestDist = legNearestP.back();
                targetLeg.start_angle = legStart.back();
                targetLeg.end_angle = legEnd.back();
                targetLeg.angle = ((targetLeg.start_angle + targetLeg.end_angle) / 2);
            } else if (TurnDir == 1) {
                targetLeg.isValid = true;
                //                targetLeg.angle = legAng.front();
                targetLeg.dist = legDist.front();
                targetLeg.nearestDist = legNearestP.front();
                targetLeg.start_angle = legStart.front();
                targetLeg.end_angle = legEnd.front();
                targetLeg.angle = ((targetLeg.start_angle + targetLeg.end_angle) / 2);
            }
            resetCounter(exitIgnoring);
            IsLegCounter = LEG_COUNTER + 1;
            resetCounter(IsNotLegCounter);
            resetCounter(rejectCounter);
            resetCounter(confirmCounter);
            robot_state = doubtTrackSTATE;
            resetCounter(exitIgnoring);
            resetCounter(exitIgnoringCounter);
        }
    }

    if (exitIgnoringCounter < (LOOP_RATE * 8)) {
        exitIgnoringCounter++;
        ROS_INFO("exitIgnoringCounter: %d", exitIgnoringCounter);
    } else {
        ROS_INFO(" **** Start Wandering ****");
        robot_state = wanderSTATE;
        resetCounter(exitIgnoring);
        resetCounter(exitIgnoringCounter);
    }
} // End of exitIgnoringFunc()

void wait4laserFunc() {
    if (last_state != wait4laserSTATE) stateTime = ros::Time::now();
    resetCounter(IsLegCounter);
    resetCounter(IsNotLegCounter);
    resetCounter(confirmCounter);
    resetCounter(rejectCounter);
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    reachTarget = false;
    start_range = 0;
    end_range = 179;
    last_state = wait4laserSTATE;
}

void wanderFunc() {
    if (last_state != wanderSTATE) {
        stateTime = ros::Time::now();
        //        robotsound_pub.publish(sound_wander);
    }
    reachTarget = false;
    start_range = 0;
    end_range = laser_ranges.size() - 1;
    resetCounter(confirmCounter);
    resetCounter(rejectCounter);
    Wander(wanderDir); // TODO: Fix Me
    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP, legStart, legEnd);
    legSelection();

    if ((last_state == doubtTrackSTATE) || ((last_IsLegCounter == IsLegCounter))) resetCounter(IsLegCounter);
//    if ((last_IsLegCounter == IsLegCounter) && (last_IsNotLegCounter < IsNotLegCounter)) resetCounter(IsLegCounter);
//    if ((last_IsLegCounter < IsLegCounter) && (last_IsNotLegCounter == IsNotLegCounter)) resetCounter(IsNotLegCounter);
    if ((last_IsLegCounter == IsLegCounter) && (LEG_COUNTER < IsNotLegCounter)) resetCounter(IsLegCounter);
    if ((LEG_COUNTER < IsLegCounter) && (last_IsNotLegCounter == IsNotLegCounter)) resetCounter(IsNotLegCounter);

    if (IsLegCounter > LEG_COUNTER) {
        robot_state = doubtTrackSTATE;
        resetCounter(IsNotLegCounter);
    }
    if (IsNotLegCounter > LEG_COUNTER) {
        robot_state = wanderSTATE;
    }
    last_state = wanderSTATE;
    last_IsLegCounter = IsLegCounter;
    last_IsNotLegCounter = IsNotLegCounter;
} // End of wanderFunc

void doubtFunc() {
    if (targetLeg.start_angle < 10) start_range = 0;
    else start_range = targetLeg.start_angle - 10;
    end_range = targetLeg.end_angle + 10;
    if (end_range > LASER_DATA -1) end_range = LASER_DATA -1;

    if (last_state != doubtTrackSTATE) {
        stateTime = ros::Time::now();
        //        robotsound_pub.publish(sound_findLeg);
    }

    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP, legStart, legEnd);
    legSelection();
    followLeg();

    if ((last_confirmCounter < confirmCounter) && (last_rejectCounter == rejectCounter)) resetCounter(rejectCounter);
    if ((last_rejectCounter < rejectCounter) && (last_confirmCounter == confirmCounter)) resetCounter(confirmCounter);
    if ((last_state == wanderSTATE) || (last_IsNotLegCounter == IsNotLegCounter)) IsNotLegCounter = 0;
//    if ((last_IsLegCounter == IsLegCounter) && (last_IsNotLegCounter < IsNotLegCounter)) resetCounter(IsLegCounter);
    if ((last_IsLegCounter == IsLegCounter) && (LEG_COUNTER < IsNotLegCounter)) resetCounter(IsLegCounter);
//    if ((last_IsLegCounter < IsLegCounter) && (last_IsNotLegCounter == IsNotLegCounter)) resetCounter(IsNotLegCounter);
    if ((LEG_COUNTER < IsLegCounter) && (last_IsNotLegCounter == IsNotLegCounter)) resetCounter(IsNotLegCounter);

    if (IsLegCounter > LEG_COUNTER) {
        insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(255, 255, 0), 35); // Yellow
        if (reachTarget) {

            updateKinectConfirm();
            ROS_INFO("IsKinectConfirm: %d", IsKinectConfirm);
            if (IsKinectConfirm) {
                confirmCounter++;
            } else {
                rejectCounter++;
            }

            if (confirmCounter > (LOOP_RATE * 0.5)) { // RIGHT TARGET
                robot_state = wait4CommandSTATE;
                resetCounter(wait4CommandCounter);
                targetLeg.isValid = true;
                resetCounter(rejectCounter);
                robotsound_pub.publish(sound_findUser); // I can see you (Rank 1)
                soundRank = 1;
            }
            if (rejectCounter > (LOOP_RATE * 4)) { // WRONG TARGET

                robot_state = ignoreSTATE;
                ignoreLeg.angle = targetLeg.angle;
                targetLeg.isValid = false;
                resetCounter(confirmCounter);
                resetCounter(rejectCounter);
                robotsound_pub.publish(sound_ignore); // Ignore (Rank 1)
                soundRank = 1;
                if (TurnDir == 1) wanderDir = 1;
                else wanderDir = 0;
            }
        } else { // Does not reach the user yet
            robot_state = doubtTrackSTATE;
        }
    }
    uint8_t switchCounter = 10;
    if ((IsNotLegCounter > LEG_COUNTER + switchCounter) || ((IsLegCounter < LEG_COUNTER) && (IsNotLegCounter < LEG_COUNTER + switchCounter))) {
        robot_state = wanderSTATE;
        if (cmd_vel.angular.z > 0.0) wanderDir = 1;
        else wanderDir = 0;
        resetCounter(IsLegCounter);
    }
    last_state = doubtTrackSTATE;
    last_IsNotLegCounter = IsNotLegCounter;
    last_IsLegCounter = IsLegCounter;
} // End of doubtFunc

void ignoreFunc() {
    resetCounter(IsLegCounter);
    resetCounter(rejectCounter);
    resetCounter(confirmCounter);
    reachTarget = false;
    if (last_state != ignoreSTATE) stateTime = ros::Time::now();
    if (wanderDir == 0) {
        start_range = 0;
        end_range = ignoreLeg.angle - 5;
    } else {
        start_range = ignoreLeg.angle + 5;
        end_range = laser_ranges.size() - 1;
    }
    Wander(wanderDir);
    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP, legStart, legEnd);
    legSelection();
    exitIgnoringFunc();
    last_state = ignoreSTATE;
} // End of ignoreFunc

void wait4CommandFunc() {
    if (last_state != wait4CommandSTATE) stateTime = ros::Time::now();
    // stop the robot
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.x = 0.0;
    
    resetCounter(rejectCounter);
    if (targetLeg.start_angle < 10) start_range = 0;
    else start_range = targetLeg.start_angle - 10;
    end_range = targetLeg.end_angle + 10;
    if (end_range > LASER_DATA-1) end_range = LASER_DATA-1;
    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP, legStart, legEnd);
    legSelection();
    insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(255, 165, 0), 35); // ORANGE

    if ((targetLeg.angle < 120.0 && targetLeg.angle > 60.0) && (targetLeg.dist < 6.0 && targetLeg.dist > 1.5)) {
        updateKinectConfirm();
        if (IsKinectConfirm) {
            confirmCounter++;
        } else {
            rejectCounter++;
        }
    }
    if ((last_confirmCounter < confirmCounter) && (last_rejectCounter == rejectCounter)) resetCounter(rejectCounter);

   if ((last_rejectCounter < rejectCounter) && (last_confirmCounter == confirmCounter)) resetCounter(confirmCounter);
   uint8_t switchCounter = 10;
   if ((IsNotLegCounter > LEG_COUNTER + switchCounter) || (rejectCounter > switchCounter)) {
        robot_state = wanderSTATE;
        IsLegCounter = 0;
    }
    if (wait4CommandCounter < (LOOP_RATE * 12)) {
        ROS_INFO("WAITING FOR GESTURE....   %d", wait4CommandCounter);
        wait4CommandCounter++;

        for (uint8_t i = 1; i < 7; i++) {
            if ((wait4CommandCounter == (LOOP_RATE * 2 * i))) {
                if(soundRank >= 4){
                    robotsound_pub.publish(sound_noGest); // I can't see gesture (Rank 4)
                    soundRank = 4;
                }
            }
        }

        for (int j = 1; j < 7; j++){
            if ((wait4CommandCounter == (LOOP_RATE * (2 * j + 1)))) {
                if (jointConf.data == 1) {
                    //pSad = false;
                    //if (!pHappy) {
                    if(soundRank >= 2){
                        robotsound_pub.publish(sound_allJoints); // I can see Joints (Rank 2)
                        soundRank = 2;
                    }
                        //pHappy = true;
                   // }
                } else {
                    //pHappy = false;
                   // if (!pSad) {
                    if(soundRank >= 2){
                        robotsound_pub.publish(sound_noJoints); // I can't see Joints (Rank 2)
                        soundRank = 2;
                    }
                       // pSad = true;
                    //}
                }
            }
        }

        if ((gest.data == "reach right") || (gest.data == "reach left")) {
            ROS_INFO(" **** Gesture Detected ****");
            ROS_INFO(" **** Go to reach user state ****");
            robot_state = reachUserSTATE;
            resetCounter(wait4CommandCounter);
            if(soundRank >= 3){
                robotsound_pub.publish(sound_reachGesture); // I saw a gesture (Rank 3)
                soundRank = 3;
            }
        } else if (gest.data == "point left") {
            ROS_INFO(" **** Gesture Detected ****");
            ROS_INFO(" **** Pointing to Left ****");
            robot_state = ignoreSTATE;
            ignoreLeg.angle = targetLeg.angle;
            resetCounter(wait4CommandCounter);
            wanderDir = 0; // Rotate Right
            if(soundRank >= 3){
                robotsound_pub.publish(sound_pointGesture); // I saw a gesture (Rank 3)
                soundRank = 3;
            }
        } else if (gest.data == "point right") {
            ROS_INFO(" **** Gesture Detected ****");
            ROS_INFO(" **** Pointing to Right ****");
            robot_state = ignoreSTATE;
            ignoreLeg.angle = targetLeg.angle;
            resetCounter(wait4CommandCounter);
            wanderDir = 1; // Rotate Left
            if(soundRank >= 3){
                robotsound_pub.publish(sound_pointGesture); // I saw a gesture (Rank 3)
                soundRank = 3;
            }
        }
    } else {
        ROS_INFO("NO GESTURE DETECTED!");
        resetCounter(wait4CommandCounter);
        robot_state = ignoreSTATE;
        if (TurnDir == 1) wanderDir = 1;
        else wanderDir = 0;

        robotsound_pub.publish(sound_ignore); // Ignore (Rank 1)
        soundRank = 1;
        ignoreLeg.angle = targetLeg.angle;
        targetLeg.isValid = false;
    }
    last_state = wait4CommandSTATE;
} // End of wait4CommandFunc

void reachUser(int user_angle, float user_dist) {

    float e_angle;
    float e_distance;
    float K2 = 6.0; //threshold
    //    e_angle = targetLeg.angle - ROBOT_HEAD_ANGLE;
    //    e_distance = targetLeg.dist - 0.6;
    e_angle = (float) user_angle - ROBOT_HEAD_ANGLE;
    e_distance = user_dist - 0.40;
    //    ROS_INFO("e_distance: %f",e_distance);
    //    ROS_INFO("e_angle: %f",e_angle);
    cmd_vel.angular.z = ((1 / (1 + exp(-e_angle / 12.0))) - 0.5);
    insertPoint(user_dist * 1000.0, user_angle, CV_RGB(255, 105, 180), 35); // Hot Pink

    if (fabs(e_angle) < K2) {
        cmd_vel.linear.x = 0.5 * ((1 / (1 + exp(-e_distance * 2))) - 0.5); // TODO: change me!
        cmd_vel.angular.z = 0.0; // stop rotating

        if ((fabs(e_distance) < 0.3) || (fabs(e_angle) > 20.0)) {
            ready2get = true;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            if ((!gripper_state.grip.inner_beam) && (!gripper_state.grip.outer_beam)) { // without object -- > get the object
                ROS_INFO("I am ready to get the object.");
                if(soundRank >= 5){
                    robotsound_pub.publish(sound_reachuser); // I'm ready to give/get the object (Rank 5)
                    soundRank = 5;
                }
                insertPoint(user_dist * 1000.0, user_angle, CV_RGB(255, 0, 0), 35); // RED
                robot_state = gettingObjectSTATE;
                gripper_control.grip.state = 1;
                gripper_control.grip.dir = 1;
                gripper_cmd = true;
                last_gripper_cmd = gripper_control.grip.dir;
            } else {
                ROS_INFO("I am ready to give the object.");
                if(soundRank >= 5){
                    robotsound_pub.publish(sound_reachuser);  // I'm ready to give/get the object (Rank 5)
                    soundRank = 5;
                }
                insertPoint(user_dist * 1000.0, user_angle, CV_RGB(250, 128, 114), 35); // SALMON
                robot_state = givingObjectSTATE;
            }
        }
    }
} //End of reachUser function

void reachUserFunc() {
    if (last_state != reachUserSTATE) stateTime = ros::Time::now();
    uint16_t user_angle, user_angle1, user_angle2, user_angle3, user_angle4,user_angle5;
    float middle = targetLeg.angle;
    float user_dist, user_dist1, user_dist2,user_dist3,user_dist4,user_dist5;
    nearestPoint(laser_ranges,start_range,end_range,user_dist5,user_angle5);
    nearestPoint(laser_ranges,start_range,middle,user_dist3,user_angle3);
    nearestPoint(laser_ranges,middle,end_range,user_dist4,user_angle4);
    nearestPoint(laser_ranges, targetLeg.start_angle, middle, user_dist1, user_angle1);
    nearestPoint(laser_ranges, middle, targetLeg.end_angle, user_dist2, user_angle2);

//    user_dist = (user_dist1 + user_dist2) / 2;
    user_angle = (user_angle1 + user_angle2) / 2;
    user_dist = user_dist5;
    start_range = targetLeg.start_angle - 10;
    end_range = targetLeg.end_angle + 10;
    reachUser(user_angle, user_dist);
    last_state = reachUserSTATE;
} //End of reachUserFunc()

void gripperGetObject() {
    ROS_INFO("last gripper command: %d", last_gripper_cmd);
    if ((gripper_state.grip.left_contact) && (gripper_state.grip.right_contact)) { // if Close
        if ((!gripper_state.grip.outer_beam) && (!gripper_state.grip.inner_beam)) { // WO object
            gripper_control.grip.state = 1;
            gripper_control.grip.dir = 1;
            gripper_cmd = true;
            if (last_gripper_cmd == 1) {
                gripper_cmd = false;
            }
            last_gripper_cmd = gripper_control.grip.dir;
        } else { // With object
            gripper_cmd = false;
        }
    } else { // if Open
        if ((!gripper_state.grip.outer_beam) && (!gripper_state.grip.inner_beam)) { // WO Object
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
} // End of gripperGetObject()

void gripperGiveObject() {
    if (givingCounter < (LOOP_RATE)*2) {
        givingCounter++;
        giveOK = false;
        if ((gripper_state.grip.left_contact) && (gripper_state.grip.right_contact)) { // if Close -- > open anyway
            gripper_control.grip.state = 1;
            gripper_control.grip.dir = 1;
            gripper_cmd = true;
            if (last_gripper_cmd == 1) gripper_cmd = false;
            last_gripper_cmd = gripper_control.grip.dir;

        } else { // if Open  -- > stay 
            gripper_cmd = false;
        }
    } else {
        resetCounter(givingCounter);
        giveOK = true;
    }
} // End of gripperGiveObject()

void getObjectFunc() {
    if (last_state != gettingObjectSTATE) stateTime = ros::Time::now();
    if (wait4ObjectCounter < (LOOP_RATE * 10)) {
        ROS_INFO("WAITING FOR OBJECT....   %d", wait4ObjectCounter);
        wait4ObjectCounter++;
        gripperGetObject();
        if (((gripper_state.grip.outer_beam) || (gripper_state.grip.inner_beam)) && (gripper_state.grip.left_contact) && (gripper_state.grip.right_contact)) {
            ROS_INFO(" **** Got the Object ****");
            ROS_INFO(" **** Backing Off ****");
            //            robot_state = backOffSTATE;
            robot_state = ignoreSTATE;
            if (TurnDir == 1) wanderDir = 1;
            else wanderDir = 0;
            if(soundRank >= 6){
                robotsound_pub.publish(sound_noProblem); // No Problem (Rank 6)
                soundRank = 6;
            }
            ignoreLeg.angle = targetLeg.angle;
            resetCounter(wait4ObjectCounter);
        }
    } else {
        ROS_INFO("NO OBJECT RECIEVED!");
        ROS_INFO(" **** Backing Off ****");
        resetCounter(wait4ObjectCounter);
        //        gripper_control.grip.state = 2;
        //        gripper_control.grip.dir = -1;
        //        gripper_cmd = true;
        //        robot_state = backOffSTATE; // go to your previous position
        robot_state = ignoreSTATE;
        if (TurnDir == 1) wanderDir = 1;
        else wanderDir = 0;
        robotsound_pub.publish(sound_ignore); // ignore (Rank 1)
        soundRank = 1;
        ignoreLeg.angle = targetLeg.angle;
        targetLeg.isValid = false;
    }
    last_state = gettingObjectSTATE;
} //END of  getObjectFunc

void giveObjectFunc() {
    if (last_state != givingObjectSTATE) stateTime = ros::Time::now();
    if (wait4ObjectCounter < (LOOP_RATE * 10)) {
        ROS_INFO("WAITING FOR SOMEONE TAKING THE OBJECT....   %d", wait4ObjectCounter);
        wait4ObjectCounter++;
        gripperGiveObject();
        //                if ((!gripper_state.grip.outer_beam) && (!gripper_state.grip.inner_beam) && (!gripper_state.grip.left_contact) && (!gripper_state.grip.right_contact)) {
        if (giveOK) {
            ROS_INFO(" **** The Object was taken ****");
            robot_state = ignoreSTATE;
            if (TurnDir == 1) wanderDir = 1;
            else wanderDir = 0;
            if(soundRank >= 6){
                robotsound_pub.publish(sound_noProblem); // No Problem (Rank 6)
                soundRank = 6;
            }
            ignoreLeg.angle = targetLeg.angle;
            resetCounter(wait4ObjectCounter);
        }
    } else {
        ROS_INFO("OBJECT IS NOT TAKEN!");
        ROS_INFO(" **** Start Ignoring...****");
        resetCounter(wait4ObjectCounter);
        //        gripper_control.grip.state = 2;
        //        gripper_control.grip.dir = -1;
        //        gripper_cmd = true;
        //        robot_state = backOffSTATE; // go to your previous position
        robot_state = ignoreSTATE;
        if (TurnDir == 1) wanderDir = 1;
        else wanderDir = 0;
        robotsound_pub.publish(sound_ignore); // ignore (Rank 1)
        soundRank = 1;
        ignoreLeg.angle = targetLeg.angle;
        targetLeg.isValid = false;
    }
    last_state = givingObjectSTATE;
} // End of giveObjectFunc()

void backOff() {
    if (last_state != backOffSTATE) stateTime = ros::Time::now();
    float er_angle;
    float er_distance;
    float K2 = 6.0; //threshold
    if (targetLeg.dist < 1.00) {
        start_range = targetLeg.start_angle - 10;
        end_range = targetLeg.end_angle + 10;
    } else if (targetLeg.dist < 2.00) {
        start_range = targetLeg.start_angle - 10;
        end_range = targetLeg.end_angle + 10;
    } else {
        start_range = targetLeg.start_angle - 10;
        end_range = targetLeg.end_angle + 10;
    }
    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP, legStart, legEnd);
    legSelection();
    er_angle = targetLeg.angle - ROBOT_HEAD_ANGLE;
    er_distance = targetLeg.dist - 3.00;
    cmd_vel.angular.z = ((1 / (1 + exp(-er_angle / 12.0))) - 0.5);
    insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(160, 32, 240), 35); // Purple 

    if (fabs(er_angle) < K2) {
        cmd_vel.linear.x = 0.5 * ((1 / (1 + exp(-er_distance * 2))) - 0.5); // TODO: change me!
        cmd_vel.angular.z = 0.0; // stop rotating

        if ((fabs(er_distance) < 0.3) || (fabs(er_angle) > 20.0)) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_INFO("I am ignoring the user.");
            robot_state = ignoreSTATE;
            if (TurnDir == 1) wanderDir = 1;
            else wanderDir = 0;
            ignoreLeg.angle = targetLeg.angle;
        }
    }
    last_state = backOffSTATE;
} // End of backOff()

void obstacle_avoidance() {
    float a = 0.3;
    float b = 0.4;
    float r[LASER_DATA];
    uint16_t safeAngle = 0;

    for (uint16_t i = 0; i < LASER_DATA; i++) {
        r[i] = sqrt((a * cos(i * PI / 180))*(a * cos(i * PI / 180)) + (b * sin(i * PI / 180))*(b * sin(i * PI / 180)));
    }
    if (laser_ranges.size() > 0) {
        for (uint16_t i = 0; i < LASER_DATA; i++) {
            //            ROS_INFO("r[%d] = %f",i*10, r[i*10]); 
            if ((laser_ranges.at(i) < r[i]) && ((cmd_vel.angular.z != 0.0) || (cmd_vel.linear.x != 0.0))) {
                ROS_INFO(" *********** DANGER ***********");
                ROS_INFO("laser_ranges.at(%d) = %f", i, laser_ranges.at(i));
                cmd_vel.angular.z = 0.0;
                cmd_vel.linear.x = 0.0;

                if (!pEStop) {
                    if(soundRank >= 7){
                        robotsound_pub.publish(sound_ES); // ES (Rank 7)
                        soundRank = 7;
                    }
                    pEStop = true;
//                    ROS_ERROR("pEStop: False ---> True");
                }
            }
            if ((laser_ranges.at(i) > r[i])) {
                safeAngle++;
            }
        }
        if (safeAngle == LASER_DATA) {
            if (pEStop) {
                pEStop = false;
//                ROS_ERROR("pEStop: True ---> False");
            }
        }
    }
}



// *********************  CALL BACK FUNCTIONS
void sonar_cb(const p2os_driver::SonarArray & msg) {
   // msg.get_ranges_vec(sonar_ranges);
    sonar_ranges = msg.ranges;
    //    ROS_INFO("SONAR DATA CAME");
}

void laser_cb(const sensor_msgs::LaserScan & msg) {
    //    ROS_INFO("LASER DATA CAME");
    clearVisWindow();
    laser_ranges.clear();
    ls_count++;
    lastLaserTime = ros::Time::now();
    //now_range = msg.ranges;
    //msg.get_ranges_vec(now_range);
    //if (now_range.size() > 0) {
    if (msg.ranges.size() > 0) {
        //msg.get_ranges_vec(laser_ranges);
        laser_ranges = msg.ranges;
        //Plot laser data
        for (size_t i = 0; i < laser_ranges.size(); i++) {
            insertPoint(laser_ranges.at(i) * 1000.0, (float) i, CV_RGB(255, 255, 255));
        }

    }
}

void gesture_cb(const std_msgs::String & msg) {
    //    ROS_INFO("GESTURE DATA CAME");
    gest = msg;
}

//void skeleton_cb(const pi_tracker::Skeleton &msg) {
//    Persons = msg;
//    lastPersonTime = ros::Time::now();
////    ROS_INFO("Skeleton DATA CAME");
//}

void user_cb(const pi_tracker::Skeleton & msg) {
    Persons = msg;
    lastPersonTime = ros::Time::now();
}

void gripper_cb(const p2os_driver::GripperState & msg) {
    gripper_state = msg;
} // End of gripper_cb

void controlCHandler(int signal) {
    should_exit = true;
}

void soundDiag_cb (const diagnostic_msgs::DiagnosticArray & msg){
    soundDiag = msg;
    // soundDiag.get_status_vec(Status);
    Status = soundDiag.status;
    soundStatus = Status.front();
}

void jointConf_cb(const std_msgs::Int32 & msg){
    jointConf = msg;
}


// *************************** main function

int main(int argc, char **argv) {
    ros::init(argc, argv, "pioneerHRI", ros::init_options::NoSigintHandler);
    signal(SIGABRT, &controlCHandler);
    signal(SIGTERM, &controlCHandler);
    signal(SIGINT, &controlCHandler);
    ros::NodeHandle n;
    ros::Rate loopRate(LOOP_RATE);
    lastSkelTime = ros::Time::now() - ros::Duration(3600);
    lastPersonTime = ros::Time::now() - ros::Duration(3600);
    lastLaserTime = ros::Time::now() - ros::Duration(3600);
    stateTime = ros::Time::now();
    last_state = wait4laserSTATE;
    robot_state = wait4laserSTATE;


    //    n.param<string > ("play_req", play_req, "Please raise your hand.");
    n.param<string > ("play_findLeg", play_findLeg, "/home/autolab/ros/workspace/RobotSounds/I/IAha1Vol.ogg");
    n.param<string > ("play_ignore", play_ignore, "/home/autolab/ros/workspace/RobotSounds/I/IFailure2Vol20.ogg");
    n.param<string > ("play_aha", play_aha, "/home/autolab/ros/workspace/RobotSounds/I/IWohoo1Vol20.ogg");
    n.param<string > ("play_reachgesture", play_reachgesture, "/home/autolab/ros/workspace/RobotSounds/I/IAttention1Vol20.ogg");
    n.param<string > ("play_noGest", play_noGest, "/home/autolab/ros/workspace/RobotSounds/I/ISad1Vol20.ogg");
    n.param<string > ("play_wander", play_wander, "/home/autolab/ros/workspace/RobotSounds/I/IThink1Vol.ogg");
    n.param<string > ("play_reachuser", play_reachuser, "/home/autolab/ros/workspace/RobotSounds/I/IGotIt1Vol.ogg");
    n.param<string > ("play_pointgesture", play_pointgesture, "/home/autolab/ros/workspace/RobotSounds/I/IAha3Vol.ogg");
    n.param<string > ("play_np", play_np, "/home/autolab/ros/workspace/RobotSounds/I/INoProb1Vol.ogg");
    n.param<string > ("play_es", play_es, "/home/autolab/ros/workspace/RobotSounds/I/IBatWarn1Vol.ogg");
    n.param<string > ("play_allJoints", play_allJoints, "/home/autolab/ros/workspace/RobotSounds/I/IHm1Vol.ogg");
    n.param<string > ("play_noJoints", play_noJoints, "/home/autolab/ros/workspace/RobotSounds/I/IExcuseMe1Vol.ogg");

    ros::Subscriber sonar_sub = n.subscribe("sonar", 50, sonar_cb);
    ros::Subscriber laser_sub = n.subscribe("scan", 50, laser_cb);
    ros::Subscriber conf_sub = n.subscribe("jointConf", 50, jointConf_cb);
    //    ros::Subscriber skeleton_sub = n.subscribe("skeleton", 50, skeleton_cb);
    ros::Subscriber user_sub = n.subscribe("user", 50, user_cb);
    ros::Subscriber gesture_sub = n.subscribe("gesture", 50, gesture_cb);
    ros::Subscriber gripper_sub = n.subscribe("gripper_state", 50, gripper_cb);
    ros::Subscriber soundDiag_sub = n.subscribe("diagnostics", 50, soundDiag_cb);
    ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist > ("cmd_vel", 5);
    //ros::Publisher debug_pub = n.advertise<std_msgs::String > ("state_debug", 5);
    ros::Publisher gripper_pub = n.advertise<p2os_driver::GripperState > ("gripper_control", 5);
    ros::Publisher robotState_pub = n.advertise<std_msgs::String > ("robotState", 10);
    userID_pub = n.advertise<std_msgs::Int32 > ("userID", 5);
    robotsound_pub = n.advertise<sound_play::SoundRequest > ("robotsound", 1);

    sound_findUser.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_findUser.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_findUser.arg = play_aha;
    sound_findLeg.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_findLeg.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_findLeg.arg = play_findLeg;
    sound_reachGesture.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_reachGesture.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_reachGesture.arg = play_reachgesture;
    sound_pointGesture.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_pointGesture.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_pointGesture.arg = play_pointgesture;
    sound_ignore.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_ignore.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_ignore.arg = play_ignore;
    sound_noGest.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_noGest.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_noGest.arg = play_noGest;
    sound_wander.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_wander.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_wander.arg = play_wander;
    sound_reachuser.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_reachuser.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_reachuser.arg = play_reachuser;
    sound_noProblem.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_noProblem.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_noProblem.arg = play_np;
    sound_ES.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_ES.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_ES.arg = play_es;
    sound_allJoints.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_allJoints.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_allJoints.arg = play_allJoints;
    sound_noJoints.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_noJoints.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_noJoints.arg = play_noJoints;

    namedWindow(lw_window);
    clearVisWindow();
    float fps_ts[FPS_BUF_SIZE];
    unsigned int counter = 0;
    ros::Time _now = ros::Time::now();

    while (ros::ok()) {
        float sum = 0.0;
        ros::Duration diffSkel = _now - lastSkelTime;
        ros::Duration diffPerson = _now - lastPersonTime;
        ros::Duration diffLaser = _now - lastLaserTime;
        ros::Duration diffStateTime = _now - stateTime;
        std_msgs::String msg_robotState;
        stringstream str_robotState;
        soundRankCounter++ ;
        if(soundRankCounter > LOOP_RATE / 2){
            resetCounter(soundRankCounter);
            soundRank = 10;
        }

        if (diffLaser.toSec() > 2.0) {
            robot_state = wait4laserSTATE;
            ROS_INFO("No laser data more than 1 sec.");
        } else {
            if (robot_state == wait4laserSTATE) {
                robot_state = wanderSTATE;
            }
        }

        if (diffStateTime.toSec() > STATE_TIME) {
            stateTime = ros::Time::now();
            robot_state = wait4laserSTATE;
            ROS_INFO("No state change in %d seconds!", STATE_TIME);
        }

        plotAllLegs();
        if ((diffSkel.toSec() > 1.0) && (go_cb != 0)) {
            resetCounter(go_cb);
            resetCounter(exUser);
        }

        if ((diffPerson.toSec() > 2.0)) {
            resetCounter(confirmCounter);
            // ROS_INFO("temp 1");
        }

        // ----------- VISUALIZATION ------------
        if (show_viz) visualizeLaser();
        fps_ts[counter] = (ros::Time::now() - _now).toNSec();
        _now = ros::Time::now();
        counter = (counter + 1) % FPS_BUF_SIZE;
        for (int i = 0; i < FPS_BUF_SIZE; i++) {
            sum += fps_ts[i];
        }
        fps = 1e9 / (sum / ((double) FPS_BUF_SIZE));
        // ----------- VISUALIZATION ------------



        if (robot_state == wait4laserSTATE) { // if there is laser data
            wait4laserFunc(); //Do Nothing!
        } else if (robot_state == wanderSTATE) {
            wanderFunc();
        } else if (robot_state == doubtTrackSTATE) {
            doubtFunc();
        } else if (robot_state == ignoreSTATE) {
            ignoreFunc();
        } else if (robot_state == wait4CommandSTATE) {
            wait4CommandFunc();
        } else if (robot_state == reachUserSTATE) {
            reachUserFunc();
        } else if (robot_state == gettingObjectSTATE) {
            getObjectFunc();
        } else if (robot_state == backOffSTATE) {
            backOff();
        } else if (robot_state == givingObjectSTATE) {
            giveObjectFunc();
        }

        last_confirmCounter = confirmCounter;
        last_rejectCounter = rejectCounter;
        if(cmd_vel.angular.z > 0) TurnDir = 1;
        else if (cmd_vel.angular.z < 0) TurnDir = 0;


        ROS_INFO("***************************************************");
        ROS_INFO("Robot State:    >>> %s <<<", state_names[robot_state].c_str());
        //ROS_INFO("LegCounters: %d       %d", IsLegCounter, IsNotLegCounter);
        // ROS_INFO("cmd_vel: x:%6.2f    z:%6.2f", cmd_vel.linear.x, cmd_vel.angular.z);
        // ROS_INFO("targetLeg.dist: %f     targetLeg.angle: %f", targetLeg.dist, targetLeg.angle);
        //ROS_INFO("targetLeg.start_angle: %d     targetLeg.end_angle: %d", targetLeg.start_angle, targetLeg.end_angle);
        //ROS_INFO("FOV: %d    %d", start_range, end_range);
        //ROS_INFO("confrimCounter: %d", confirmCounter);
        //ROS_INFO("rejectCounter: %d", rejectCounter);
       // ROS_INFO("kinectConfirm: %d", IsKinectConfirm);
        //ROS_INFO("Gesture: %s", gest.data.c_str());
        //ROS_INFO("exitIgnoring: %d", exitIgnoring);
        // ROS_INFO("diffStateTime: %f", diffStateTime.toSec());
        //ROS_ERROR("Message sound: %s ",soundStatus.message.c_str());

        if(soundStatus.message == "1 sounds playing"){
            ROS_INFO("HAPPY!");
        }

        str_robotState << state_names[robot_state].c_str();
        msg_robotState.data = str_robotState.str();
        robotState_pub.publish(msg_robotState);
        obstacle_avoidance();
        cmd_vel_pub_.publish(cmd_vel);
        if (gripper_cmd) {
            gripper_pub.publish(gripper_control);
            gripper_cmd = false;
        }
        if (should_exit) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            for (int i = 0; i < 10; i++) {
                cmd_vel_pub_.publish(cmd_vel);
            }
            ros::spinOnce();
            loopRate.sleep();
            ros::shutdown();
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
} //End of main 
