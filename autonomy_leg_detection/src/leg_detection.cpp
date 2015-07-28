#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <miarn/feature_geometry.h>
#include <miarn/feature_person.h>
#include <angles/angles.h>
#include "polarcord.h"
#include <visualization_msgs/Marker.h>

#define _USE_MATH_DEFINES
#define FPS_BUF_SIZE 10
// each pixel is 20x20 mm^2
#define PIXEL_RATIO 20
#define _USE_MATH_DEFINES
#define M2MM_RATIO 1000.
#define DEBUG true

/********************************************************
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

********************************************************/

ros::Publisher leg_pub;
ros::Publisher marker_pub;
LaserFeatureX laserFeature;
FeatureLegTracker featureLegTracker;
geometry_msgs::PoseArray global_legs;
sensor_msgs::LaserScan ldata;
bool show_marker;
double marker_scale;

uint8_t segment_size(const LaserFeatureX::segment &seg){return (seg.end - seg.begin + 1);}

float segments_distance(LaserFeatureX::segment &right, LaserFeatureX::segment &left)
{
    int right_legmid = (int) (right.begin + right.end)/2;
    int left_legmid = (int) (left.begin + left.end)/2;
    return ( distance( laserFeature.point_xy.at(right_legmid), laserFeature.point_xy.at(left_legmid)));
}

inline float distanceXY(float x1, float y1, float x2, float y2)
{
    return(sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)));
}

bool FindLegPatterns(){

    // CHECK IF TWO CONSECUTIVE SEGMENTS HAVE SAME NUMBER OF BEAMS
    // AND CHECK IF THE DISTANCE BETWEEN THE MIDDLE POINTS OF TWO CANDIDATE SEGMENTS
    //IS LESS THAN TWO*MAX LEG DIAMETER

    bool find_leg_patterns = false;
    if(laserFeature.segments.empty()) return find_leg_patterns;

    for(size_t i = 0; i < laserFeature.segments.size()-1; i++)
    {
        if(abs(segment_size(laserFeature.segments.at(i))) < 2) continue;

        geometry_msgs::Pose tmp_leg_pose;
        size_t right_index, left_index;
        right_index = i;
        left_index = i+1;
        LaserFeatureX::segment right = laserFeature.segments.at(right_index);
        LaserFeatureX::segment left = laserFeature.segments.at(left_index);

        int right_legmid = int(right.begin + right.end)/2; // index of right leg mid point
        int left_legmid = int(left.begin + left.end)/2; // index of left leg mid point

        //ADDTION TO MIARN LEG-DETECTOR

        // CHECK IF THERE IS A PATTERN OF FALL-...-RISE-...-FALL-...-RISE IN A SEGMENT THAT ITS SIZE IS BIGGER THAN 4

        bool first_fall = false;
        bool last_rise = false;
        bool inside_fall = false;
        bool inside_rise = false;
        unsigned int right_segment_size = abs(segment_size(right));
        float range_derivative [right_segment_size-1];
        bool slope_sign [right_segment_size-1];
        bool line_sign [right_segment_size - 2]; // TODO: CHANGE THE NAME

        if(right_segment_size > 4){
            for(size_t j = 0; j < right_segment_size-1; j++)
            {
                range_derivative[j] = (laserFeature.ranges.at(right.begin + j + 1) - laserFeature.ranges.at(right.begin + j));
                slope_sign[j] = (laserFeature.ranges.at(right.begin + j + 1) > laserFeature.ranges.at(right.begin + j))
                        ? true : false;
            }

            //            if(range_derivative[0] < 0.0) first_fall = true;
            //            if(range_derivative[right_segment_size-2] > 0.0 ) last_rise = true;
            if(slope_sign[0] == false) first_fall = true;
            if(slope_sign[right_segment_size-2] == true ) last_rise = true;

            //            for(size_t k = 1; k < right_segment_size-2; k++){
            //                if(range_derivative[k] < 0.0){inside_fall = true;}
            //                if(range_derivative[k] > 0.0){inside_rise = true;}
            //            }

            uint test_counter = 0;

            for(size_t k = 0; k < right_segment_size-2; k++){
                line_sign[k] = (slope_sign[k] == slope_sign[k+1]) ? true : false;
                if (line_sign[k] == false) test_counter++;
            }
            //            if(first_fall && last_rise && inside_fall && inside_rise &&
            //                    distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) < (int)laserFeature.max_leg_diameter &&
            //                    test_counter %2 == 1 && test_counter > 1 ){
            double two_leg_dist = distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end));
            double leg_right_dist = distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right_legmid));
            double leg_reft_dist = distance(laserFeature.point_xy.at(right_legmid),laserFeature.point_xy.at(right.end));
            if(first_fall && last_rise && test_counter %2 == 1 && test_counter > 1 &&
                    two_leg_dist < laserFeature.max_leg_diameter * 2 &&
                    two_leg_dist > laserFeature.min_leg_diameter * 2 &&
                    leg_right_dist < laserFeature.max_leg_diameter &&
                    leg_reft_dist < laserFeature.max_leg_diameter &&
                    leg_right_dist > laserFeature.min_leg_diameter &&
                    leg_reft_dist > laserFeature.min_leg_diameter){
                tmp_leg_pose.position.x = laserFeature.point_xy.at(right_legmid).x/M2MM_RATIO;
                tmp_leg_pose.position.y = laserFeature.point_xy.at(right_legmid).y/M2MM_RATIO;
                tmp_leg_pose.position.z = 0.0;
                global_legs.poses.push_back(tmp_leg_pose);
                find_leg_patterns = true;
                //                 ROS_ERROR("1     x: %.2f     y:%.2f", tmp_leg_pose.position.x, tmp_leg_pose.position.y);
                //                 ROS_ERROR("counter:  %d", test_counter);
                //                 ROS_ERROR("2 legs:  %.2f", two_leg_dist);
                //                 ROS_ERROR("leg:  %.2f       %.2f", leg_reft_dist, leg_right_dist);
                //                 ROS_INFO("begin: %d   middle: %d    end: %d", right.begin, right_legmid, right.end);

            }
        }
        //END OF ADDITION

        if((abs(segment_size(right) - segment_size(left)) < 5) &&
                (segments_distance(right, left) < 2 * laserFeature.max_leg_diameter)&&
                distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) < laserFeature.max_leg_diameter &&
                distance(laserFeature.point_xy.at(left.begin),laserFeature.point_xy.at(left.end)) < laserFeature.max_leg_diameter &&
                distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) > laserFeature.min_leg_diameter &&
                distance(laserFeature.point_xy.at(left.begin),laserFeature.point_xy.at(left.end)) > laserFeature.min_leg_diameter)
        {
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
            find_leg_patterns = true;
            //            ROS_INFO("2     x: %.2f     y:%.2f",laserFeature.point_xy.at(right_legmid).x/M2MM_RATIO, laserFeature.point_xy.at(right_legmid).y/M2MM_RATIO);
            //            ROS_INFO("2     x: %.2f     y:%.2f",laserFeature.point_xy.at(left_legmid).x/M2MM_RATIO, laserFeature.point_xy.at(left_legmid).y/M2MM_RATIO);

        } else
            if(i < laserFeature.segments.size()-2){
                left_index = i + 2;
                left = laserFeature.segments.at(left_index);
                left_legmid = int(left.begin + left.end)/2; // index of left leg mid point

                if((abs(segment_size(right) - segment_size(left)) < 5) &&
                        (segments_distance(right, left) < 2 * laserFeature.max_leg_diameter)&&
                        distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) < laserFeature.max_leg_diameter &&
                        distance(laserFeature.point_xy.at(left.begin),laserFeature.point_xy.at(left.end)) < laserFeature.max_leg_diameter &&
                        distance(laserFeature.point_xy.at(right.begin),laserFeature.point_xy.at(right.end)) > laserFeature.min_leg_diameter &&
                        distance(laserFeature.point_xy.at(left.begin),laserFeature.point_xy.at(left.end)) > laserFeature.min_leg_diameter)
                {

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
                    find_leg_patterns = true;
                    //                    ROS_INFO("3     x: %.2f     y:%.2f",laserFeature.point_xy.at(right_legmid).x/M2MM_RATIO, laserFeature.point_xy.at(right_legmid).y/M2MM_RATIO);
                    //                    ROS_INFO("3     x: %.2f     y:%.2f",laserFeature.point_xy.at(left_legmid).x/M2MM_RATIO, laserFeature.point_xy.at(left_legmid).y/M2MM_RATIO);

                }
            }
    }

    return find_leg_patterns;
}


void publishLegs()
{
    if(ldata.ranges.empty()) return;

//    ROS_INFO("========== SPINNING =============");

    float r, b, db;

    geometry_msgs::PoseArray legs, publish_legs;
    geometry_msgs::Pose tmpl;

    laserFeature.fdata.clear();
    laserFeature.ranges.clear();
    laserFeature.segments.clear();
    laserFeature.point_xy.clear();
    featureLegTracker.fdata_out.clear();
    featureLegTracker.fdata.clear();


    laserFeature.max_laser_range = ldata.range_max * M2MM_RATIO;
    b = ldata.angle_min;
    db = ldata.angle_increment;

    for(size_t i = 0; i < ldata.ranges.size() ; i++)
    {
        if(ldata.ranges.at(i) > ldata.range_max)
            r = ldata.range_max * M2MM_RATIO;

        else if(ldata.ranges.at(i) < ldata.range_min)
            r = ldata.range_min * M2MM_RATIO;

        else
            r = ldata.ranges.at(i)*M2MM_RATIO;

        laserFeature.ranges.push_back(r);

        xy temp;
        temp.x = r * cos(b);
        temp.y = r * sin(b);
        laserFeature.point_xy.push_back(temp);
        b += db;
    }

    // SEGMENTS LASER DATA
    laserFeature.Segmentation();
    assert(laserFeature.segments.size() < laserFeature.point_xy.size());

    // CHECK FOR LEG PATTERNS (MIARN)
    for(size_t i = 0; i < laserFeature.segments.size(); i++)
    {
        bool check2legs = 1;
        check2legs = laserFeature.FitArc(laserFeature.segments.at(i).begin, laserFeature.segments.at(i).end);
        bool asghar = laserFeature.FindLeg(laserFeature.segments.at(i).begin, laserFeature.segments.at(i).end, check2legs);
    }

    // CHECK FOR LEG PATTERNS (MY ALGORITHM)
    global_legs.poses.clear();
    bool test = FindLegPatterns();

    for(size_t i = 0; i < global_legs.poses.size(); i++)
    {
        legs.poses.push_back(global_legs.poses.at(i));
    }

/******************************************************************************
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
******************************************************************************/

    // TRACK THE LEGS AND PERSONS

    featureLegTracker.fdata.clear();
    featureLegTracker.fdata.assign(laserFeature.fdata.begin(), laserFeature.fdata.end());

    featureLegTracker.LCReset();
    featureLegTracker.CreateIncoming();
    featureLegTracker.LegMatchInTime();
    featureLegTracker.LegClean();
    featureLegTracker.LegCreate();
    featureLegTracker.PersonUpdate();
    featureLegTracker.PersonCreate();
    featureLegTracker.FillFeature();

    for (size_t i = 0; i < featureLegTracker.fdata_out.size(); i++)
    {

        if(featureLegTracker.fdata_out.at(i).type == MIARN_FEATURE_TYPE_PERSON ||
                featureLegTracker.fdata_out.at(i).type == MIARN_FEATURE_TYPE_LEG)
        {
            xy pose1, pose2;
            pose1.x = featureLegTracker.fdata_out.at(i).pos[0]/ M2MM_RATIO;
            pose1.y = featureLegTracker.fdata_out.at(i).pos[1]/ M2MM_RATIO;

            tmpl.position.x = pose1.x ;
            tmpl.position.y = pose1.y;
            tmpl.position.z = 0.0;

            bool new_leg = true;

            for(size_t k = 0; k < legs.poses.size(); k++)
            {
                pose2.x = legs.poses.at(k).position.x;
                pose2.y = legs.poses.at(k).position.y ;

                new_leg = new_leg && (distance(pose1,pose2) > 1.0);

                if (!new_leg) continue;
            }

            if(new_leg)
            {
                legs.poses.push_back(tmpl);
                /*insertPoint(sqrt(pose1.x*pose1.x + pose1.y*pose1.y), atan2(pose1.y,pose1.x),
                                                CV_RGB(255, 0, 0),lw_height,lw_width, 20);*/
            }

        }

    }


    for(size_t i = 0; i < legs.poses.size(); i++)
    {
        bool new_leg = true;
        geometry_msgs::Pose pose1 = legs.poses.at(i);

        for(size_t j = 0; j < publish_legs.poses.size(); j++){
            geometry_msgs::Pose pose2 = publish_legs.poses.at(j);

            new_leg = (new_leg && (distanceXY(pose1.position.x, pose1.position.y,pose2.position.x, pose2.position.y)
                                 > laserFeature.max_leg_diameter/M2MM_RATIO));
        }
        if(new_leg) publish_legs.poses.push_back(legs.poses.at(i));
    }

    publish_legs.header = ldata.header;
    leg_pub.publish(publish_legs);

    // Publish Leg Markers

    if(show_marker)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Point p;

        marker.header.frame_id = publish_legs.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "leg_markers";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker_scale;
        marker.scale.y = marker_scale;
        marker.scale.z = 0.25;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0);

        for(size_t i = 0; i < publish_legs.poses.size(); i++)
        {
            p.x = publish_legs.poses.at(i).position.x;
            p.y = publish_legs.poses.at(i).position.y;
            p.z = 0;
            marker.points.push_back(p);
        }

        marker_pub.publish(marker);
    }
}

void laser_cb(const sensor_msgs::LaserScan & msg)
{
    if (msg.ranges.empty())
        return;
    ldata = msg;

}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"leg_detection");
    ros::NodeHandle n;
    ROS_INFO("Starting Leg Detection ...");
    ros::Rate loopRate(10);
    ros::Time now = ros::Time::now();

    double arc_min_aperture, arc_max_aperture, arc_std_max, segmentation_threshold;
    double line_min_distance, line_error_threshold, max_leg_diameter, min_leg_diameter;
    double leg_clean_ticks, person_clean_ticks, leg_update_radius, person_radius;
    ros::param::param("~/laserFeature/arc_min_aperture",arc_min_aperture, angles::from_degrees(10.0));
    ROS_INFO("laserFeature/arc_min_aperture is set to %.2lf",arc_min_aperture);

    ros::param::param("~/laserFeature/arc_max_aperture",arc_max_aperture, angles::from_degrees(20.0));
    ROS_INFO("laserFeature/arc_max_aperture is set to %.2lf",arc_max_aperture);

    ros::param::param("~/laserFeature/arc_std_max",arc_std_max,0.2);
    ROS_INFO("laserFeature/arc_std_max is set to %.2lf", arc_std_max);

    ros::param::param("~/laserFeature/segmentation_threshold", segmentation_threshold,500.0);
    ROS_INFO("laserFeature/segmentation_threshold is set to %.2lf", segmentation_threshold);

    ros::param::param("~/laserFeature/line_min_distance",line_min_distance,170.0);
    ROS_INFO("laserFeature/line_min_distance is set to %.2lf",line_min_distance);

    ros::param::param("~/laserFeature/line_error_threshold",line_error_threshold,5.0);
    ROS_INFO("laserFeature/line_error_threshold is set to %.2lf",line_error_threshold);

    ros::param::param("~/laserFeature/max_leg_diameter",max_leg_diameter,300.0);
    ROS_INFO("laserFeature/max_leg_diameter is set ti %.2lf",max_leg_diameter);

    ros::param::param("~/laserFeature/min_leg_diameter",min_leg_diameter,30.0);
    ROS_INFO("laserFeature/min_leg_diameter is set to %.2lf",min_leg_diameter);

    ros::param::param("~/featureLegTracker/leg_clean_ticks",leg_clean_ticks,5.0);
    ROS_INFO("featureLegTracker/leg_clean_ticks is set to %.2lf",leg_clean_ticks);

    ros::param::param("~/featureLegTracker/person_clean_ticks",person_clean_ticks,5.0);
    ROS_INFO("featureLegTracker/person_clean_ticks is set to %.2lf",person_clean_ticks);

    ros::param::param("~/featureLegTracker/leg_update_radius",leg_update_radius,100.0);
    ROS_INFO("featureLegTracker/leg_update_radius is set to %.2lf",leg_update_radius);

    ros::param::param("~/featureLegTracker/person_radius",person_radius,300.0);
    ROS_INFO("featureLegTracker/person_radius is set to %.2lf",person_radius);

    ros::param::param("~/show_marker",show_marker,true);
    ros::param::param("~/marker_scale",marker_scale,0.5);


    laserFeature.arc_min_aperture = arc_min_aperture;
    laserFeature.arc_max_aperture = arc_max_aperture;
    laserFeature.arc_std_max = arc_std_max;
    laserFeature.segmentation_threshold = segmentation_threshold;
    laserFeature.line_min_distance = line_min_distance;
    laserFeature.line_error_threshold = line_error_threshold;
    laserFeature.max_leg_diameter = max_leg_diameter;
    laserFeature.min_leg_diameter = min_leg_diameter;

    featureLegTracker.leg_clean_ticks = leg_clean_ticks;
    featureLegTracker.person_clean_ticks = person_clean_ticks;
    featureLegTracker.leg_update_radius  =leg_update_radius;
    featureLegTracker.person_radius = person_radius;

    featureLegTracker.segmentation_threshold = laserFeature.segmentation_threshold;

/******************************************************************************
    ros::param::param("~show_viz",show_viz,false);
    ROS_INFO("show_vis set to %d",show_viz);
    if(show_viz){
        cv::namedWindow(lw_window);
        clearVisWindow(1000.0,1000.0);
    }
    float fps_ts[FPS_BUF_SIZE];
    unsigned int counter = 0;
******************************************************************************/


    ros::Subscriber laser_sub = n.subscribe("scan", 10, laser_cb);
    leg_pub = n.advertise<geometry_msgs::PoseArray>("legs",10);
    marker_pub = n.advertise<visualization_msgs::Marker>("leg/marker", 1);

    while(ros::ok()){

 /******************************************************************************
        // ----------- LASER VISUALIZATION ------------
        float sum = 0.0;
        if (show_viz) visualizeLaser();
        fps_ts[counter] = (ros::Time::now() - now).toNSec();
        now = ros::Time::now();
        counter = (counter + 1) % FPS_BUF_SIZE;
        for (unsigned int i = 0; i < FPS_BUF_SIZE; i++)
            sum += fps_ts[i];
        fps = 1e9 / (sum / ((double) FPS_BUF_SIZE));
        // -----------LASER VISUALIZATION ------------
******************************************************************************/

        publishLegs();
        ros::spinOnce();

        if(loopRate.cycleTime() > loopRate.expectedCycleTime())
            ROS_ERROR("LEG DETECTION: It is taking too long!");
        if(!loopRate.sleep())
            ROS_ERROR("LEG DETECTION: Not enough time left");

    }

    return 0;
}

