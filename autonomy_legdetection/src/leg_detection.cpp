#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <algorithm>
#include <assert.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include "miarn/feature_geometry.h"
#include "miarn/feature_person.h"


#define _USE_MATH_DEFINES
#define FPS_BUF_SIZE 10
// each pixel is 20x20 mm^2
#define PIXEL_RATIO 20


// //#include "polarcord.h"
#define _USE_MATH_DEFINES
#define M2MM_RATIO 1000.0

#define DEBUG true

class PolarPose
{
    //        x
    //        ^
    //        |
    //  y <----
public:
    float range;
    float angle;
    PolarPose():range(0.0), angle(0.0){;}
    PolarPose(const float r, const float a ):range(r), angle(a){;}
    inline void fromCart(const float x, const float y)
    {
        range = sqrt((x * x) + (y * y));
        angle = atan2(y,x);
    }
    inline void toCart(float &x, float &y) {
        x = range * cos(angle);
        y = range * sin(angle);
    }
};
// //#include "polarcord.h"


// **************************** LASER-VISUALIZATION (start)
float fps;
bool show_viz;

std::string lw_window = "laser";
cv::Mat laser_vis;


void visualizeLaser() {
    //cv::flip(laser_vis,laser_vis,1);
    cv::imshow(lw_window, laser_vis);
    cv::waitKey(1);
}


void clearVisWindow(const float lw_height, const float lw_width) {
    laser_vis = cv::Mat::zeros(lw_height, lw_width, CV_8UC3);
    char buff[25];
    sprintf(buff, "%4.2f fps", fps);
    std::string str = buff;
    cv::putText(laser_vis, str, cv::Point(20, 20), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
}

void insertPoint(float r,
                 float th,
                 const cv::Scalar& color,
                 const float lw_height,
                 const float lw_width,
                 uint16_t rad = 1,
                 uint8_t thickness = 1)
{

    PolarPose pr(r, th);
    float x_mm, y_mm;


    pr.toCart(x_mm, y_mm);

    float x_px = x_mm / PIXEL_RATIO;
    float y_px = y_mm / PIXEL_RATIO;

    //    ---> x_vis (row)
    //    |
    //    v
    //    y_vis(col)

    float row = -x_px + lw_height / 2.0;
    float col = y_px + lw_width / 2.0;

    circle(laser_vis, cv::Point(col, row), rad, color, thickness);
}

// **************************** LASER-VISUALIZATION (end)

ros::Publisher leg_pub;
LaserFeatureX laserFeature;
FeatureLegTracker featureLegTracker;

unsigned int segment_size(const LaserFeatureX::segment &seg){return (seg.begin - seg.end + 1);}

float segments_distance(LaserFeatureX::segment &right, LaserFeatureX::segment &left){
    int right_legmid = int(right.begin + right.end)/2;
    int left_legmid = int(left.begin + left.end)/2;
    return (distance(laserFeature.point_xy.at(right_legmid), laserFeature.point_xy.at(left_legmid)));
}

bool FindLegPatterns(){

    // CHECK IF TWO CONSECUTIVE SEGMENTS HAVE SAME NUMBER OF BEAMS
    // AND CHECK IF THE DISTANCE BETWEEN THE MIDDLE POINTS OF TWO CANDIDATE SEGMENTS
    //IS LESS THAN TWO*MAX LEG DIAMETER

    if(laserFeature.segments.empty()) return 0;

    for(size_t i = 0; i < laserFeature.segments.size()-1; i++){
        size_t right_index, left_index;
        right_index = i;
        left_index = i+1;
        LaserFeatureX::segment right = laserFeature.segments.at(right_index);
        LaserFeatureX::segment left = laserFeature.segments.at(left_index);

        if((abs(segment_size(right) - segment_size(left)) < 3) &&
               (segments_distance(right, left) < 2*laserFeature.max_leg_diameter)&&
                distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) < (int)laserFeature.max_leg_diameter &&
                distance(laserFeature.point_xy.at(left.begin),laserFeature.point_xy.at(left.end)) < (int)laserFeature.max_leg_diameter)
        {
            int right_legmid = int(right.begin + right.end)/2; // index of right leg mid point
            int left_legmid = int(left.begin + left.end)/2; // index of left leg mid point

            //ADDPROBABLELEGS
            laserFeature.AddProbableLeg(
                        laserFeature.point_xy.at(right_legmid).x, laserFeature.point_xy.at(right_legmid).y,
                        laserFeature.point_xy.at(right.begin).x, laserFeature.point_xy.at(right.end).x,
                        laserFeature.point_xy.at(right.begin).y, laserFeature.point_xy.at(right.end).y,
                        right.begin,right.end);
            laserFeature.AddProbableLeg(
                        laserFeature.point_xy.at(left_legmid).x, laserFeature.point_xy.at(left_legmid).y,
                        laserFeature.point_xy.at(left.begin).x, laserFeature.point_xy.at(left.end).x,
                        laserFeature.point_xy.at(left.begin).y, laserFeature.point_xy.at(left.end).y,
                        left.begin, left.end);
            return 1;
        } else
            if(i < laserFeature.segments.size()-2) left_index = i + 2;
    }
    return 0;
}

void laser_cb(const sensor_msgs::LaserScan & msg)
{
    if (msg.ranges.empty())
        return;

    float r, b, db;
    laserFeature.fdata.clear();
    laserFeature.ranges.clear();
    laserFeature.segments.clear();
    laserFeature.point_xy.clear();
    featureLegTracker.fdata_out.clear();
    featureLegTracker.fdata.clear();

    laserFeature.max_laser_range = msg.range_max*M2MM_RATIO;
    b = msg.angle_min;
    db = msg.angle_increment;

    for(size_t i = 0; i < msg.ranges.size() ; i++){
        if(msg.ranges.at(i) > msg.range_max)
            r = msg.range_max * M2MM_RATIO;
        else
            r = msg.ranges.at(i)*M2MM_RATIO;
        laserFeature.ranges.push_back(r);

        xy temp;
        temp.x = r * cos(b);
        temp.y = r * sin(b);
        laserFeature.point_xy.push_back(temp);
        b += db;
    }

    // Segments laser data
    laserFeature.Segmentation();
    assert(laserFeature.segments.size() < laserFeature.point_xy.size());
    // look for leg patterns
    for(size_t i = 0; i < laserFeature.segments.size(); i++){
        bool check2legs = 1;
        check2legs = !laserFeature.FitArc(laserFeature.segments.at(i).begin, laserFeature.segments.at(i).end);
        laserFeature.FindLeg(laserFeature.segments.at(i).begin, laserFeature.segments.at(i).end, check2legs);
    }
    FindLegPatterns();

    //+++++++++++++++ PLOT LASER DATA
    //show_viz = true;
    float laser_max_range = msg.range_max; // 20.0
    if(show_viz) laser_max_range = 10.0;
    int lw_width = (int) (laser_max_range * M2MM_RATIO / PIXEL_RATIO) * 2;
    int lw_height = (int) (laser_max_range * M2MM_RATIO / PIXEL_RATIO) * 2;
    clearVisWindow(lw_height,lw_width);

    b = msg.angle_min;
    for (size_t i = 0; i < msg.ranges.size(); i++){
        insertPoint(msg.ranges.at(i) * M2MM_RATIO, b, CV_RGB(255, 255, 255),lw_height,lw_width,1,2);
        b += db;
    }
    insertPoint(2 * M2MM_RATIO, msg.angle_max, CV_RGB(255, 0, 0),lw_height,lw_width,1,10);
    insertPoint(2 * M2MM_RATIO, msg.angle_min, CV_RGB(0, 0, 255),lw_height,lw_width,1,10);
    insertPoint(0.01 * M2MM_RATIO, 0, CV_RGB(0, 255, 0),lw_height,lw_width,1,10);
    //--------------- PLOT LASER DATA



    geometry_msgs::PoseArray legs, tmp_legs;
    geometry_msgs::Pose tmp_leg_pose;

// TRACK THE LEGS AND PERSONS

    featureLegTracker.fdata.clear();
    for(size_t i = 0; i < laserFeature.fdata.size(); i++)
        featureLegTracker.fdata.push_back(laserFeature.fdata.at(i));

    featureLegTracker.LCReset();
    featureLegTracker.CreateIncoming();
    featureLegTracker.LegMatchInTime();
    featureLegTracker.LegClean();
    featureLegTracker.LegCreate();
    featureLegTracker.PersonUpdate();
    featureLegTracker.PersonCreate();
    featureLegTracker.FillFeature();

    size_t legcount;
    for (size_t i = 0; i < featureLegTracker.fdata_out.size(); i++) {
        if(featureLegTracker.fdata_out.at(i).type == MIARN_FEATURE_TYPE_PERSON) {
            legcount ++;
            xy pose1, pose2;
            pose1.x = featureLegTracker.fdata_out.at(i).pos[0];
            pose1.y = featureLegTracker.fdata_out.at(i).pos[1];

            tmp_leg_pose.position.x = pose1.x;
            tmp_leg_pose.position.y = pose1.y;
            tmp_leg_pose.position.z = 0.0;

            if(!legs.poses.empty()){
                pose2.x = legs.poses.back().position.x;
                pose2.y = legs.poses.back().position.y;

                // check if this is acually a new pair of legs!
                if(distance(pose1,pose2) > laserFeature.max_leg_diameter*2){
                    legs.poses.push_back(tmp_leg_pose);
                    insertPoint(sqrt(pose1.x*pose1.x + pose1.y*pose1.y), atan2(pose1.y,pose1.x), CV_RGB(255, 0, 0),lw_height,lw_width, 20);
                }
            }
        } else if(featureLegTracker.fdata_out.at(i).type == MIARN_FEATURE_TYPE_LEG) {
            legcount ++;
            xy pose1;

            pose1.x = featureLegTracker.fdata_out.at(i).pos[0];
            pose1.y = featureLegTracker.fdata_out.at(i).pos[1];

        tmp_leg_pose.position.x = pose1.x;
        tmp_leg_pose.position.y = pose1.y;
        tmp_leg_pose.position.z = 0.0;
        tmp_legs.poses.push_back(tmp_leg_pose);
        //insertPoint(sqrt(pose1.x*pose1.x + pose1.y*pose1.y), atan2(pose1.y,pose1.x), CV_RGB(0, 0, 255),lw_height,lw_width, 20);
        }
    }

    //check if there is a pair of legs in leg data
    if(!tmp_legs.poses.empty()){
        xy pose1, pose2;
        for(size_t i = 0; i < tmp_legs.poses.size()-1; i++){

            tmp_leg_pose = tmp_legs.poses.at(i);
            pose1.x = tmp_leg_pose.position.x;
            pose1.y = tmp_leg_pose.position.y;
            tmp_leg_pose = tmp_legs.poses.at(i+1);
            pose2.x = tmp_leg_pose.position.x;
            pose2.y = tmp_leg_pose.position.y;
            if(distance(pose1,pose2) < laserFeature.max_leg_diameter &&
                    distance(pose1,pose2) > laserFeature.min_leg_diameter){

                tmp_leg_pose.position.x = (pose1.x + pose2.x)/2/M2MM_RATIO;
                tmp_leg_pose.position.y = (pose1.y + pose2.y)/2/M2MM_RATIO;
                tmp_leg_pose.position.z = 0.0;
                legs.poses.push_back(tmp_leg_pose);
                insertPoint(sqrt(tmp_leg_pose.position.x*tmp_leg_pose.position.x + tmp_leg_pose.position.y*tmp_leg_pose.position.y),
                            atan2(tmp_leg_pose.position.y,tmp_leg_pose.position.x), CV_RGB(255, 0, 0),lw_height,lw_width, 20);

            }
        }
    }
    legs.header.frame_id = msg.header.frame_id;
    legs.header.stamp = ros::Time::now();

    leg_pub.publish(legs);
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"leg_detection");
    ros::NodeHandle n;
    ROS_INFO("Starting Leg Detection ...");
    ros::Rate loopRate(10);
    ros::Time _now = ros::Time::now();

    double _arc_min_aperture, _arc_max_aperture, _arc_std_max, _segmentation_threshold;
    double _line_min_distance, _line_error_threshold, _max_leg_diameter, _min_leg_diameter;
    double _leg_clean_ticks, _person_clean_ticks, _leg_update_radius, _person_radius;
    ros::param::param("~/laserFeature/arc_min_aperture",_arc_min_aperture,1.57);
    ROS_INFO("laserFeature/arc_min_aperture is set to %.2lf",_arc_min_aperture);

    ros::param::param("~/laserFeature/arc_max_aperture",_arc_max_aperture,2.35);
    ROS_INFO("laserFeature/arc_max_aperture is set to %.2lf",_arc_max_aperture);

    ros::param::param("~/laserFeature/arc_std_max",_arc_std_max,0.2);
    ROS_INFO("laserFeature/arc_std_max is set to %.2lf", _arc_std_max);

    ros::param::param("~/laserFeature/segmentation_threshold", _segmentation_threshold,500.0);
    ROS_INFO("laserFeature/segmentation_threshold is set to %.2lf", _segmentation_threshold);

    ros::param::param("~/laserFeature/line_min_distance",_line_min_distance,170.0);
    ROS_INFO("laserFeature/line_min_distance is set to %.2lf",_line_min_distance);

    ros::param::param("~/laserFeature/line_error_threshold",_line_error_threshold,5.0);
    ROS_INFO("laserFeature/line_error_threshold is set to %.2lf",_line_error_threshold);

    ros::param::param("~/laserFeature/max_leg_diameter",_max_leg_diameter,300.0);
    ROS_INFO("laserFeature/max_leg_diameter is set ti %.2lf",_max_leg_diameter);

    ros::param::param("~/laserFeature/min_leg_diameter",_min_leg_diameter,30.0);
    ROS_INFO("laserFeature/min_leg_diameter is set to %.2lf",_min_leg_diameter);

    ros::param::param("~/featureLegTracker/leg_clean_ticks",_leg_clean_ticks,5.0);
    ROS_INFO("featureLegTracker/leg_clean_ticks is set to %.2lf",_leg_clean_ticks);

    ros::param::param("~/featureLegTracker/person_clean_ticks",_person_clean_ticks,5.0);
    ROS_INFO("featureLegTracker/person_clean_ticks is set to %.2lf",_person_clean_ticks);

    ros::param::param("~/featureLegTracker/leg_update_radius",_leg_update_radius,100.0);
    ROS_INFO("featureLegTracker/leg_update_radius is set to %.2lf",_leg_update_radius);

    ros::param::param("~/featureLegTracker/person_radius",_person_radius,300.0);
    ROS_INFO("featureLegTracker/person_radius is set to %.2lf",_person_radius);

    laserFeature.arc_min_aperture = _arc_min_aperture;
    laserFeature.arc_max_aperture = _arc_max_aperture;
    laserFeature.arc_std_max = _arc_std_max;
    laserFeature.segmentation_threshold = _segmentation_threshold;
    laserFeature.line_min_distance = _line_min_distance;
    laserFeature.line_error_threshold = _line_error_threshold;
    laserFeature.max_leg_diameter = _max_leg_diameter;
    laserFeature.min_leg_diameter = _min_leg_diameter;

    featureLegTracker.leg_clean_ticks = _leg_clean_ticks;
    featureLegTracker.person_clean_ticks = _person_clean_ticks;
    featureLegTracker.leg_update_radius  =_leg_update_radius;
    featureLegTracker.person_radius = _person_radius;

    featureLegTracker.segmentation_threshold = laserFeature.segmentation_threshold;

    ros::param::param("~show_viz",show_viz,false);
    ROS_INFO("show_vis set to %d",show_viz);


    ros::Subscriber laser_sub = n.subscribe("scan", 10, laser_cb);
    leg_pub = n.advertise<geometry_msgs::PoseArray>("legs",10);

    cv::namedWindow(lw_window);
    clearVisWindow(1000.0,1000.0);
    float fps_ts[FPS_BUF_SIZE];
    unsigned int counter = 0;

    while(ros::ok()){

        // ----------- LASER VISUALIZATION ------------
        float sum = 0.0;
        if (show_viz) visualizeLaser();
        fps_ts[counter] = (ros::Time::now() - _now).toNSec();
        _now = ros::Time::now();
        counter = (counter + 1) % FPS_BUF_SIZE;
        for (unsigned int i = 0; i < FPS_BUF_SIZE; i++)
            sum += fps_ts[i];
        fps = 1e9 / (sum / ((double) FPS_BUF_SIZE));
        // -----------LASER VISUALIZATION ------------

        ros::spinOnce();
        if(loopRate.cycleTime() > loopRate.expectedCycleTime())
            ROS_ERROR("It is taking too long!");
        if(!loopRate.sleep())
            ROS_ERROR("Not enough time left");
    }
    return 0;
}

