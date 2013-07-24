// LAST UPDATE July 25 2012
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
#include "p2os_driver/SonarArray.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#define FPS_BUF_SIZE 10

//Raw Laser data
vector<float> laser_ranges;

bool show_viz = true;
float fps;

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

const std::string lw_window = "laser";
cv::Mat laser_vis;
const int lw_width = 900;
const int lw_height = 500;
const int lw_mm_width = 18000; //mm
const int lw_mm_height = 9000; //mm

void visualizeLaser() {
    imshow(lw_window, laser_vis);
    waitKey(1);
}

void clearVisWindow() {
    laser_vis = Mat::zeros(lw_height, lw_width, CV_8UC3);
    char buff[25];
    sprintf(buff, "%4.2f fps", fps);
    std::string str = buff;
    putText(laser_vis, str, Point(20, 20), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
}

/**
 * 
 * @param r     : mm
 * @param th    : degree (0 standard) 
 * @param color 
 * @param size  : in px
 */
void insertPoint(float r, float th, const Scalar& color, int rad = 2) {
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
#define DISTANCE 3.00 // Meters
#define ROBOT_HEAD_ANGLE 90.0
#define WANDER_SPEED = 0.15
#define WAIT_COUNTER 50
#define LASER_MAX_RANGE 8.0

using namespace std;

geometry_msgs::Twist cmd_vel;
sound_play::SoundRequest sound_stop;
sound_play::SoundRequest sound_findUser;
sound_play::SoundRequest sound_findLeg;
sound_play::SoundRequest sound_ignore;

typedef struct _leg {
    float angle;
    float dist;
    float nearestDist;
    bool isValid;
};
bool wall = false;
bool IsTracking;
bool ignoreLeg = false;
int IsLegCounter = 0;
int IsNotLegCounter = 0;
int exUser = 0;
int go_cb = 0;
int skel_cb_counter = 0;
int say_aha = 0;
int ls_count = 0;
int rejectCounter = 0;
int confirmCounter = 0;
int exitIgnoring = 0;
int wanderDir;
int wait4CommandCounter = 0;
float pre_legAng = 0.0;
//float LASER_MAX_RANGE = 8.0;
vector<float> now_range, old_range, oldest_range;
ros::Time lastSkelTime;
ros::Time lastPersonTime;
string play_req, play_findLeg, play_ignore, play_aha;
ros::Publisher robotsound_pub;
pi_tracker::Skeleton Persons;
std_msgs::String gest;
int start_range, end_range;

// Always hold all legs regardless of state
vector<float> tempLegsDist;
vector<float> tempLegAng;
vector<float> tempLegNP;
int tempNumLegs;

// Target Leg for tracking
_leg targetLeg;

enum {
    wanderSTATE, wait4CommandSTATE, doubtTrackSTATE, ignoreSTATE, reachUserSTATE
} robot_state;

string state_names[5] = {"Wander", "ConfTrack", "DoubtTrack", "Ignoring", "Getting Object"};

int followLeg();
void nearestPoint(vector<float>, int, int, float &, int &);
void IsLeg(vector<float>, int, int, int, int, bool &);
void Wander(int);
void legSelection(const int, vector<float>, vector<float>, vector<float>);
bool kinectConfirm();
void reachUser();

void legsInLaserData(vector<float> ranges, int start_range, int end_range, int &numLegs, vector<float> &legDist, vector<float> &legAng, vector<float> &legNearestP) {

    float diffLaserData[ranges.size()];
    double threshold = 0.1;
    float filter[ranges.size()];
    unsigned int i, laserIndex, firstNeg, firstPos, secondNeg, secondPos;
    numLegs = 0;
    int LegWidth = 10;
    float Ang2Leg, PreLegPos, NearestDist;
    int nearestP;
    bool isLeg;

    numLegs = 0;
    legAng.clear();
    legDist.clear();
    legNearestP.clear();

    for (int i = 0; i < ranges.size(); i++) {
        if (ranges.at(i) >= LASER_MAX_RANGE) ranges.at(i) = LASER_MAX_RANGE;
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
    for (int k = 0; k < ranges.size() - 1; k++) {
        if ((filter[k] - filter[k + 1] > LASER_MAX_RANGE) && (filter[k] - filter[k + 1] < -LASER_MAX_RANGE)) {
            filter[k + 1] = 0.0;
        }
    }

    /* Searching for the legs */


    firstNeg = start_range;
    while (firstNeg < end_range) {
        if (filter[firstNeg] < 0.0) {
            firstPos = firstNeg + 1;
            while ((firstPos < (firstNeg + LegWidth)) && (firstPos < end_range)) {
                if (filter[firstPos] > 0.0) {
                    secondNeg = firstPos + 1;
                    while ((secondNeg < (firstPos + LegWidth)) && (secondNeg < end_range)) {
                        if (filter[secondNeg] < 0.0) {
                            secondPos = secondNeg + 1;
                            while ((secondPos < (secondNeg + LegWidth)) && (secondPos < end_range)) {
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
                                            PreLegPos = Ang2Leg;
                                            numLegs++;
                                        }
                                    } else {
                                        legDist.push_back(NearestDist);
                                        legAng.push_back(Ang2Leg);
                                        legNearestP.push_back(nearestP);
                                        PreLegPos = Ang2Leg;
                                        numLegs = 1;
                                    }
                                    firstNeg = secondPos;
                                    secondNeg = end_range;
                                    firstPos = end_range;
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

bool kinectConfirm() {
    int userNum;
    if (Persons.name.size() > 6) {
        userNum = Persons.name.size() - 15;
    } else {
        userNum = Persons.name.size();
    }
    for (int i = 0; i < Persons.position.size(); i++) {/* TODO: FIX The parameters*/
        if ((fabs(Persons.position.at(i).z / 1000.0 - targetLeg.dist) < 1.0) && (fabs(((Persons.position.at(i).x / 10.0) + 60.0) - targetLeg.angle) < 10.0) && (Persons.name.at(i) != "invalid")) {
            ROS_INFO("There is a person at %6.0f Degree !", targetLeg.angle);
            return true;
        }
    }
    return false;
} /* End of kinectConfirm */

void laser_cb(const sensor_msgs::LaserScan &msg) {

    ROS_INFO("***************************************************");
    ROS_INFO("Robot State:    >>> %s <<<", state_names[robot_state].c_str());
    ROS_INFO("cmd_vel_X: %6.2f", cmd_vel.linear.x);
    ROS_INFO("cmd_vel_Z: %6.2f", cmd_vel.angular.z);
    clearVisWindow();
    laser_ranges.clear();
    int numLegs;
    vector<float> legDist, legAng;
    vector<float> legNearestP;
    ls_count++;


    if (ls_count == 1) {
        msg.get_ranges_vec(now_range);
        old_range = now_range;
        oldest_range = now_range;
    } else if (ls_count == 2) {
        old_range = now_range;
        oldest_range = now_range;
        msg.get_ranges_vec(now_range);
    } else {
        old_range = now_range;
        oldest_range = old_range;
        msg.get_ranges_vec(now_range);
    }

    /* Weighted moving average*/
    for (int i = 0; i < LASER_DATA; i++) {
        laser_ranges.push_back((0.5 * now_range.at(i) + 0.3 * old_range.at(i) + 0.2 * oldest_range.at(i)));
    }

    /* ****** Find All Pairs of Legs in Laser Data In Proper range ***** */
    int start_range, end_range;

    if ((robot_state == wait4CommandSTATE) || (robot_state == doubtTrackSTATE || (robot_state == reachUserSTATE))) {
        if (targetLeg.dist < DISTANCE) {
            start_range = targetLeg.angle - 25;
            end_range = targetLeg.angle + 25;
        } else {
            start_range = targetLeg.angle - 10;
            end_range = targetLeg.angle + 10;
        }

    } else if (robot_state == ignoreSTATE) {
        if (wanderDir == 0) {
            start_range = 0;
            end_range = targetLeg.angle - 5;
        } else {
            start_range = targetLeg.angle + 5;
            end_range = laser_ranges.size() - 1;
        }
    } else { // Should be wandering
        start_range = 0;
        end_range = laser_ranges.size() - 1;
    }

    if (start_range < 0) start_range = 0;
    if (end_range > 179) end_range = 179;

    // TODO: Fix me w/ post filtering
    legsInLaserData(laser_ranges, 0, LASER_DATA - 1, tempNumLegs, tempLegsDist, tempLegAng, tempLegNP);
    legsInLaserData(laser_ranges, start_range, end_range, numLegs, legDist, legAng, legNearestP);
    ROS_INFO("FOV: [%d   %d]", start_range, end_range);

    /* Plot all legs in laser FOV in grey */
    for (int i = 0; i < tempNumLegs; i++) {
        insertPoint(tempLegsDist.at(i)* 1000.0, tempLegAng.at(i), CV_RGB(127, 127, 127), 20); // GREY
    }

    /* Plot legs in selected angle n green */
    for (int i = 0; i < numLegs; i++) {
        ROS_INFO("leg # %d Angle: %6.0f         Distance: %6.4f ", i + 1, legAng.at(i), legDist.at(i));
        insertPoint(legDist.at(i)* 1000.0, legAng.at(i), CV_RGB(0, 127, 0), 20);
    }


    for (int i = 0; i < laser_ranges.size(); i++) {
        insertPoint(laser_ranges.at(i) * 1000.0, (float) i, CV_RGB(255, 255, 255));
    }

    /* ****** Pick One Pair of Legs ***** */
    legSelection(numLegs, legDist, legAng, legNearestP);

} /* End of laserCallback */

void nearestPoint(vector<float> ranges, int a, int d, float & nearestDist, int & nearestP) {
    int n;
    n = a;
    for (int i = a; i < d + 1; i++) {
        if (ranges.at(i) <= ranges.at(n)) n = i;
    }
    nearestDist = ranges.at(n);
    nearestP = n;
} // End of nearestPoint Function

void IsLeg(vector<float> ranges, int a, int b, int c, int d, bool & isLeg) {

    float D1, D2;
    int P1, P2;
    nearestPoint(ranges, a, b, D1, P1);
    nearestPoint(ranges, c, d, D2, P2);
    if (fabs(D1 - D2) > 1.00) {
        isLeg = false;
    } else {
        isLeg = true;
    }
} // End of IsLeg Function

void Wander(int direction) {
    cmd_vel.linear.x = 0.0;
    float wanderSpeed = 0.2;

    if (direction == 0) {
        cmd_vel.angular.z = -wanderSpeed;
    } else {
        cmd_vel.angular.z = wanderSpeed;
    }
}

void legSelection(const int numLeg, vector<float> legDist, vector<float> legAng, vector<float> legNearestP) {

    float IsLegAngle = 10.0;
    int LegCounter = 6;
    bool inKinectFOV;
    float minAng;

    if (robot_state == ignoreSTATE) {
        Wander(wanderDir);
        IsLegCounter = 0;
        if (numLeg > 0) { /* If in Ignoring state there were a leg change the state to dounbtracking */
            exitIgnoring++;
            if (exitIgnoring > 10) {
                if (wanderDir == 0) {
                    targetLeg.isValid = true;
                    targetLeg.angle = legAng.back();
                    targetLeg.dist = legDist.back();
                    targetLeg.nearestDist = legNearestP.back();
                } else if (wanderDir == 1) {
                    targetLeg.isValid = true;
                    targetLeg.angle = legAng.front();
                    targetLeg.dist = legDist.front();
                    targetLeg.nearestDist = legNearestP.front();
                }
                exitIgnoring = 0;
                IsLegCounter = LegCounter + 1;
                IsNotLegCounter = 0;
                rejectCounter = 0;
                ROS_INFO("Temp 3");
                confirmCounter = 0;
                robot_state = doubtTrackSTATE;
                ROS_INFO("targetLeg.angle: %6.0f", targetLeg.angle);
            }
        }
    } else { /* If robot state was anything except ignoring */
        if (numLeg > 0) {
            if (numLeg == 1) {
                targetLeg.isValid = true;
                targetLeg.angle = legAng.at(0);
                targetLeg.dist = legDist.at(0);
                targetLeg.nearestDist = legNearestP.at(0);
            } else { /*if (numLeg > 1) { // Select one pair */
                inKinectFOV = false;
                for (int legIndex = 0; legIndex < numLeg; legIndex++) { // kinectFOV
                    if ((legAng.at(legIndex) < 120.0 && legAng.at(legIndex) > 60.0) && (legDist.at(legIndex) < 5.5 && legDist.at(legIndex) > 2.5)) {
                        targetLeg.isValid = true;
                        targetLeg.angle = legAng.at(legIndex);
                        targetLeg.dist = legDist.at(legIndex);
                        targetLeg.nearestDist = legNearestP.at(legIndex);
                        inKinectFOV = true;
                    }
                }
                /* Not in kinectFOV - Pick the most ahead leg and put it at the begining of array. */
                if (!inKinectFOV) {
                    int minIndex = 0;
                    minAng = fabs(legAng.at(0) - ROBOT_HEAD_ANGLE);

                    for (int legIndex = 1; legIndex < numLeg; legIndex++) {
                        if (fabs(legAng.at(legIndex) - ROBOT_HEAD_ANGLE) < minAng) {
                            minAng = fabs(legAng.at(legIndex) - ROBOT_HEAD_ANGLE);
                            minIndex = legIndex;
                        }
                    }

                    targetLeg.isValid = true;
                    targetLeg.angle = legAng.at(minIndex);
                    targetLeg.dist = legDist.at(minIndex);
                    targetLeg.nearestDist = legNearestP.at(minIndex);
                }
            }
            if (IsLegCounter == 0 || (fabs(targetLeg.angle - pre_legAng) < IsLegAngle)) IsLegCounter++;

            pre_legAng = targetLeg.angle;
            insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(0, 0, 255), 25); //BLUE

            if (IsLegCounter > LegCounter) {
                robot_state = doubtTrackSTATE;
                IsNotLegCounter = 0;
                //TODO: Sound        
                if (IsLegCounter == LegCounter + 2) {
                    sound_findLeg.sound = sound_play::SoundRequest::PLAY_FILE;
                    sound_findLeg.command = sound_play::SoundRequest::PLAY_ONCE;
                    sound_findLeg.arg = play_findLeg;
                    robotsound_pub.publish(sound_findLeg);
                }
            }
        } else { // There is nobody there!
            IsNotLegCounter++;
            if (IsNotLegCounter > LegCounter) {
                targetLeg.isValid = false;
                robot_state = wanderSTATE;
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
                /*
                 * //TODO: Sound  */
                if (IsNotLegCounter == LegCounter + 1) {
                    sound_ignore.sound = sound_play::SoundRequest::PLAY_FILE;
                    sound_ignore.command = sound_play::SoundRequest::PLAY_ONCE;
                    sound_ignore.arg = play_ignore;
                    robotsound_pub.publish(sound_ignore);
                }

            }
        }

        if (robot_state == doubtTrackSTATE || robot_state == wait4CommandSTATE) {

            if (robot_state == doubtTrackSTATE) insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(255, 165, 0), 35); // orange
            if (robot_state == wait4CommandSTATE) insertPoint(targetLeg.dist * 1000.0, targetLeg.angle, CV_RGB(255, 0, 0), 35); // RED

            if (kinectConfirm()) {
                confirmCounter++;
            } else {
                rejectCounter++;
            }
            followLeg();
            ROS_INFO("I am following the leg. Counter: %d", IsLegCounter);
        }
    }
} // End of legSelection function

void sonar_cb(const p2os_driver::SonarArray &msg) {
    vector<double> sonar_ranges;
    msg.get_ranges_vec(sonar_ranges);
    double dis;
    dis = (0 * sonar_ranges[10]) + (0.5 * sonar_ranges[11]) + (0.5 * sonar_ranges[12]) + (0 * sonar_ranges[13]);
    if (dis < 0.75) {
        wall = true;
    } else {
        wall = false;
    }
    //    ROS_INFO("Wall: %d Distance to wall:%6.4f", wall, dis);
}

void skeleton_cb(const pi_tracker::Skeleton &msg) {

    lastPersonTime = ros::Time::now();
    Persons = msg;
    float eps = 0.0001;

    if (skel_cb_counter > 3) { // The /skeleton freq. is very high, So I drop 2 out of 3.

        skel_cb_counter = 0;
        int valid_counter;

        if (Persons.name.size() > 6) {
            valid_counter = Persons.name.size() - 15;
        } else {
            valid_counter = Persons.name.size();
        }

        for (int i = 0; i < valid_counter; ++i) {

            if (fabs(Persons.position.at(i).z / 1000.0) < eps) {
                Persons.name.at(i) = "invalid";
                valid_counter = valid_counter - 1;
            }
        }

        ROS_INFO("[Kinect] I am looking at %d persons.", valid_counter);

//        if (valid_counter == 0) rejectCounter++;

        for (int i = 0; i < valid_counter; ++i) {
            if (Persons.name.at(i) != "invalid")
                ROS_INFO("[Kinect] Detected Object at: %6.0f Degree & at: %6.4f Distance", (Persons.position.at(i).x / 10.0) + 60.0, Persons.position.at(i).z / 1000.0);
        }
    }

    skel_cb_counter++;

} // End of skeleton_cb

void gesture_cb(const std_msgs::String & msg) {
    gest = msg;
    if (gest.data == "right") ROS_INFO("*** RIGHT HAND GESTURE ***");
    if (gest.data == "left") ROS_INFO("*** LEFT HAND GESTURE ***");
}

int followLeg() {

    IsTracking = true;
    float error_angle;
    float error_distance;
    float K1 = 6.0; //threshold
    bool waitTimer;
    error_angle = targetLeg.angle - ROBOT_HEAD_ANGLE;
    error_distance = targetLeg.dist - DISTANCE;
    cmd_vel.angular.z = ((1 / (1 + exp(-error_angle / 12.0))) - 0.5);
    if (fabs(error_angle) < K1) {
        
        cmd_vel.linear.x = ((1 / (1 + exp(-error_distance * 2))) - 0.5);
        
        if (cmd_vel.linear.x < 0.0 && wall) {
            cmd_vel.linear.x = 0.0;
            ROS_INFO("There is a wall at the back of robot!");
        }

        cmd_vel.angular.z = 0.0; // stop rotating

        if ((fabs(error_distance) < 0.3) || (fabs(error_angle) > 20.0)) {

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            ROS_INFO("I find target.");
            ROS_INFO("KinectConfirm: %d", kinectConfirm());
            ROS_INFO("confirmCounter: %d", confirmCounter);
            ROS_INFO("rejectCounter: %d", rejectCounter);

            if (confirmCounter > 10) {
                //TODO: FIX mE
                ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>> Right Target !");
                robot_state = wait4CommandSTATE;

                if (wait4CommandCounter < WAIT_COUNTER) {
                    ROS_INFO("WAITING FOR GESTURE....   %d", wait4CommandCounter);
                    wait4CommandCounter++;
                    if ((gest.data == "right") || (gest.data == "left")) {
                        ROS_INFO(" **** Gesture Detected ****");
                        ROS_INFO(" **** Go to get object state ****");
                        robot_state = reachUserSTATE;
                        wait4CommandCounter = 0;
                        reachUser();
                    }
                }

                if (wait4CommandCounter >= WAIT_COUNTER) {
                    ROS_INFO("NO GESTURE DETECTED!");
                    //                    waitTimer = false;
                    wait4CommandCounter = 0;
                    robot_state = ignoreSTATE;
                    wait4CommandCounter = 0;
                    targetLeg.isValid = false;
                    confirmCounter = 0;
                    rejectCounter = 0;
                    IsLegCounter = 0;
                    ignoreLeg = true;
                }
                targetLeg.isValid = true;
                rejectCounter = 0;
                // TODO: Sound
                if (say_aha == 0) {
                    sound_findUser.sound = sound_play::SoundRequest::PLAY_FILE;
                    sound_findUser.command = sound_play::SoundRequest::PLAY_ONCE;
                    sound_findUser.arg = play_aha;
                    robotsound_pub.publish(sound_findUser);
                    say_aha = 1;
                }
            }
            if (rejectCounter > 10) {
                ROS_INFO("Wrong Target !");
                targetLeg.isValid = false;
                ROS_INFO("Temp 1");
                confirmCounter = 0;
                rejectCounter = 0;
                IsLegCounter = 0;
                ignoreLeg = true;
                robot_state = ignoreSTATE;
                int leftLegs = 0;
                int rightLegs = 0;

                for (int i = 0; i < tempNumLegs; i++) {
                    rightLegs = (tempLegAng.at(i) < targetLeg.angle - 5.0) ? rightLegs + 1 : rightLegs;
                    leftLegs = (tempLegAng.at(i) > targetLeg.angle + 5.0) ? leftLegs + 1 : leftLegs;
                }
                wanderDir = rand() % 2;
                if (rightLegs > leftLegs) {
                    ROS_INFO("TURN RIGHT !: %d       %d", rightLegs, leftLegs);
                    wanderDir = 0;
                } else if (leftLegs > rightLegs) {
                    ROS_INFO("TURN LEFT !: %d       %d", leftLegs, rightLegs);
                    wanderDir = 1;
                }
                ROS_INFO("wanderDir: %d", wanderDir);
            }
            return 0;
        }
    }
    //    TODO: Sound
    if (fabs(cmd_vel.angular.z) > 0.1 || fabs(cmd_vel.linear.x) > 0.1) {
        say_aha = 0;
    }
    return 0;

} // End of followLeg function

void wait4gesture() {

} // End of wait4gesture function

void reachUser() {

    float e_angle;
    float e_distance;
    float K2 = 6.0; //threshold
    bool waitTimer;
    e_angle = targetLeg.angle - ROBOT_HEAD_ANGLE;
    e_distance = targetLeg.dist - 1.00;
    cmd_vel.angular.z = ((1 / (1 + exp(-e_angle / 12.0))) - 0.5);
    if (fabs(e_angle) < K2) {
        cmd_vel.linear.x = ((1 / (1 + exp(-e_distance * 2))) - 0.5);
        cmd_vel.angular.z = 0.0; // stop rotating

        if ((fabs(e_distance) < 0.3) || (fabs(e_angle) > 20.0)) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            ROS_INFO("I am ready to get the object.");
        }
    }

} //End of reachUser function

// *************************** main function

int main(int argc, char **argv) {
    ros::init(argc, argv, "findLegs");
    ros::NodeHandle n;
    ros::Rate loopRate(25);
    lastSkelTime = ros::Time::now() - ros::Duration(3600);
    lastPersonTime = ros::Time::now() - ros::Duration(3600);
    robot_state = wanderSTATE;
    cmd_vel.linear.y = cmd_vel.angular.z = cmd_vel.linear.x = 0.0;
    n.param<std::string > ("play_req", play_req, "Please raise your hand.");
    n.param<std::string > ("play_findLeg", play_findLeg, "/home/autolab/ros/ros_workspace/RobotSounds/I/IHey2.ogg");
    n.param<std::string > ("play_aha", play_aha, "/home/autolab/ros/ros_workspace/RobotSounds/I/IWohoo1.ogg");
    n.param<std::string > ("play_lostLeg", play_ignore, "/home/autolab/ros/ros_workspace/RobotSounds/I/IFailure2.ogg");
    ros::Subscriber sonar_sub = n.subscribe("sonar", 50, sonar_cb);
    ros::Subscriber sub = n.subscribe("scan", 50, laser_cb);
    ros::Subscriber skeleton_sub = n.subscribe("skeleton", 50, skeleton_cb);
    ros::Subscriber gesture_sub = n.subscribe("gesture", 50, gesture_cb);
    ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist > ("cmd_vel", 100);
    ros::Publisher debug_pub = n.advertise<std_msgs::String > ("state_debug", 100);
    robotsound_pub = n.advertise<sound_play::SoundRequest > ("robotsound", 10);
    targetLeg.isValid = false;

    namedWindow(lw_window);
    clearVisWindow();
    std_msgs::String msg_debug;
    std::stringstream str_debug;
    float fps_ts[FPS_BUF_SIZE];
    int counter = 0;
    ros::Time _now = ros::Time::now();
    while (ros::ok()) {
        ros::Duration diffSkel = ros::Time::now() - lastSkelTime;
        if ((diffSkel.toSec() > 1.0) && (go_cb != 0)) {
            go_cb = 0;
            exUser = 0;
        }
        ros::Duration diffPerson = ros::Time::now() - lastPersonTime;
        if ((diffPerson.toSec() > 2.0)) {
            confirmCounter = 0;
        }
        cmd_vel_pub_.publish(cmd_vel);
        if (show_viz) visualizeLaser();
        ros::spinOnce();
        fps_ts[counter] = (ros::Time::now() - _now).toNSec();
        _now = ros::Time::now();
        counter = (counter + 1) % FPS_BUF_SIZE;
        float sum = 0.0;
        for (int i = 0; i < FPS_BUF_SIZE; i++) {
            sum += fps_ts[i];
        }
        fps = 1e9 / (sum / ((double) FPS_BUF_SIZE));

        str_debug.str("");
        str_debug << "S: " << state_names[robot_state];
        str_debug << " Vx:" << cmd_vel.linear.x;
        str_debug << " Vz:" << cmd_vel.angular.z;
        msg_debug.data = str_debug.str();
        debug_pub.publish(msg_debug);
        loopRate.sleep();
    }
    return 0;
} //End of main 
