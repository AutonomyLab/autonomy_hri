#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pi_tracker/Skeleton.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <hark_msgs/HarkSource.h>
#include <hark_msgs/HarkSourceVal.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "lkgrid.h"


#define _USE_MATH_DEFINES
#define _PIXEL_RESOLUTION 100
#define _SHOW_GRIDS true
#define FREE_CELL_PROBABILITY 0.1

using namespace cv;

const unsigned int GRID_COLS = (_GRID_ANGLE_MAX - _GRID_ANGLE_MIN)/_GRID_ANGLE_RESOLUTION;
const unsigned int GRID_ROWS = (_GRID_RANGE_MAX - _GRID_RANGE_MIN)/_GRID_RANGE_RESOLUTION;
const unsigned int GRID_SIZE = GRID_COLS*GRID_ROWS;

Mat laser_grid_img, user_grid_img, hark_grid_img, human_grid_img;

/*Geometry_msgs/PoseArray.msg
 * std_msgs/Header header
 * geometry_msgs/Pose[] poses
 *      geometry_msgs/Pose.msg
 *          geometry_msgs/Point position (in ros coordinate frame convention)
 *          geometry_msgs/Quaternion orientation
 * /*/


LikelihoodGrid laserGrid(GRID_ROWS, GRID_COLS);
LikelihoodGrid userGrid(GRID_ROWS, GRID_COLS,0.5, 5.00, _GRID_RANGE_RESOLUTION,-30.0, 30.0, _GRID_ANGLE_RESOLUTION, 0.0);
LikelihoodGrid harkGrid(GRID_ROWS, GRID_COLS);
LikelihoodGrid humanGrid(GRID_ROWS, GRID_COLS);


void show_grid(const std::string& imgname,
               Mat img)
{
    imshow(imgname,img);
    waitKey(1);
}

void base_gridlines(Mat img,
                    const float h = _GRID_RANGE_MAX*_PIXEL_RESOLUTION,
                    const float w = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    for(unsigned int i = 0; i <= _GRID_RANGE_MAX*2; i++)
        circle(img, Point(w/2,h),(i*_GRID_RANGE_RESOLUTION)*_PIXEL_RESOLUTION,Scalar(255));
    for(unsigned int i = 1; i < _GRID_ANGLE_MAX*2/_GRID_ANGLE_RESOLUTION; i ++)
        line(img,Point(w/2,h), Point(h*cos(i*_GRID_ANGLE_RESOLUTION*M_PI/180) + (w/2),h - h*sin(i*_GRID_ANGLE_RESOLUTION*M_PI/180)),Scalar(255));
}

void clearVisWindow(Mat& img)
{
    //img = Mat::zeros(lw_height, lw_width, CV_8UC3);
    //char buff[25];
    //sprintf(buff, "%4.2f fps", fps);
    //string str = buff;
    //putText(img, str, Point(20, 20), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
}


void fill_cell(Mat &img,
               const float rr,
               const float tt,
               const float score = 1,
               const float h = _GRID_RANGE_MAX*_PIXEL_RESOLUTION,
               const float w = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    float r,t;
    Point PointArray[4];

    r = rr - _GRID_RANGE_RESOLUTION/2;
    t = tt - _GRID_ANGLE_RESOLUTION/2;
    PointArray[0].x = r*cos(t*M_PI/180)*_PIXEL_RESOLUTION + (w/2);
    PointArray[0].y = h - r*sin(t*M_PI/180)*_PIXEL_RESOLUTION;
    r = rr + _GRID_RANGE_RESOLUTION/2;
    t = tt - _GRID_ANGLE_RESOLUTION/2;
    PointArray[1].x = r*cos(t*M_PI/180)*_PIXEL_RESOLUTION + (w/2);
    PointArray[1].y = h - r*sin(t*M_PI/180)*_PIXEL_RESOLUTION;
    r = rr + _GRID_RANGE_RESOLUTION/2;
    t = tt + _GRID_ANGLE_RESOLUTION/2;
    PointArray[2].x = r*cos(t*M_PI/180)*_PIXEL_RESOLUTION + (w/2);
    PointArray[2].y = h - r*sin(t*M_PI/180)*_PIXEL_RESOLUTION;
    r = rr - _GRID_RANGE_RESOLUTION/2;
    t = tt + _GRID_ANGLE_RESOLUTION/2;
    PointArray[3].x = r*cos(t*M_PI/180)*_PIXEL_RESOLUTION + (w/2);
    PointArray[3].y = h - r*sin(t*M_PI/180)*_PIXEL_RESOLUTION;

    fillConvexPoly(img, PointArray, 4, Scalar(255*score));
}

void fill_grid(Mat &img,
               const LikelihoodGrid& grid)
{
    for(unsigned int rr = 0; rr < GRID_ROWS; rr++){
        for(unsigned int cc = 0; cc < GRID_COLS; cc++){

            float r = (rr + 0.5)*_GRID_RANGE_RESOLUTION;
            float t = (cc + 0.5)*_GRID_ANGLE_RESOLUTION;
            fill_cell(img, r,t,grid.data[rr][cc]);
        }
    }
}

//void clear_grid(Mat &img,
//                const float p = 0.0,
//                const float h = _GRID_RANGE_MAX*_PIXEL_RESOLUTION,
//                const float w = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
//{
//    img = Mat::zeros(h, w, CV_8UC3)*p;
//}
Mat create_img_grid(const float p = 0.0,
                    const float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION,
                    const float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    //float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    //float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    Mat img = Mat::ones(height,width,CV_8UC1)*p;
    base_gridlines(img);
    return img;
}

/*
void update_human_img(Mat &img, const float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION, const float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    //float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    //float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    for(unsigned int i = 0; i < ; i ++){
        for(unsigned int j = 0; j < ; j++){

        }
    }
}
*/



void legs_cb(const geometry_msgs::PoseArray &msg)
{
    std::vector<geometry_msgs::Pose> legs;
    std::vector<PolarPose> legPoses;
    if(!msg.poses.empty()){
        legs = msg.poses;
        //legPoses.clear();
        for(unsigned int i = 0; i < legs.size(); i++){
            PolarPose tempPos;
            tempPos.fromCart(legs.at(i).position.x, legs.at(i).position.y);
            legPoses.push_back(tempPos);
        }
        ROS_INFO("*** LASER ***");
        laserGrid.assign(legPoses);
        laserGrid.free_lk(FREE_CELL_PROBABILITY);
        fill_grid(laser_grid_img, laserGrid);
        base_gridlines(laser_grid_img);
    }
}

void user_cb(const pi_tracker::Skeleton &msg){
    std::vector<geometry_msgs::Vector3> user;
    std::vector<PolarPose> userPoses;
    if(!msg.position.empty()){
        user = msg.position;
        //userPoses.clear();
        for(unsigned int i = 0; i < user.size(); i++){
            PolarPose tempPos;
            if(user.at(i).z != 0){
                tempPos.fromCart(user.at(i).z/1000, user.at(i).x/1000);
                userPoses.push_back(tempPos);
            }
        }
        ROS_INFO("*** SKELETON ***");
        userGrid.assign(userPoses);
        userGrid.free_lk(FREE_CELL_PROBABILITY);
        fill_grid(user_grid_img, userGrid);
        base_gridlines(user_grid_img);
    }
}

void hark_cb(const hark_msgs::HarkSource &msg){
    std::vector<hark_msgs::HarkSourceVal> src;
    std::vector<PolarPose > harkPoses;
    if(!msg.src.empty()){
        src = msg.src;
        //harkPoses.clear();

        for(unsigned int i = 0; i < src.size(); i++){
            PolarPose tempPos;
            for(float j = 0; j < harkGrid.rangeMax; j += harkGrid.rangeRes ){

                tempPos.angle = src.at(i).azimuth;
                tempPos.range = j + (harkGrid.rangeRes)/2;
                harkPoses.push_back(tempPos);
            }
        }
        ROS_INFO("*** SOUND ***");

        //        update_likelihood(harkGrid,harkPoses);
        //        free_likelihood(harkGrid);
        harkGrid.assign(harkPoses);
        harkGrid.free_lk(FREE_CELL_PROBABILITY);
        fill_grid(hark_grid_img, harkGrid);
        base_gridlines(hark_grid_img);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    ROS_INFO("Creating the likelihood field ...");
    ros::Rate looprate(5);

    ros::Subscriber legs_sub = n.subscribe("legs",1,legs_cb);
    ros::Subscriber user_sub = n.subscribe("user",1,user_cb);
    ros::Subscriber hark_sub = n.subscribe("HarkSource", 1, hark_cb);

    laser_grid_img = create_img_grid(FREE_CELL_PROBABILITY);
    user_grid_img = create_img_grid(FREE_CELL_PROBABILITY);
    hark_grid_img = create_img_grid(FREE_CELL_PROBABILITY);
    human_grid_img = create_img_grid();


    while (ros::ok()) {


//    float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
//    float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;
//    for(unsigned int i = 0; i < height; i ++){
//        for(unsigned int j = 0; j < width; j++){

//        }
//    }


        if (_SHOW_GRIDS){
            show_grid("LASER GRID VISUALIZATION",laser_grid_img);
            show_grid("SKELETON GRID VISUALIZATION",user_grid_img);
            show_grid("HARK GRID VISUALIZATION",hark_grid_img);
            //show_grid("HUMAN GRID VISUALIZATION",human_grid_img);
        }

        ros::spinOnce();
        if(looprate.cycleTime() > looprate.expectedCycleTime())
            ROS_ERROR("It is taking too long!");
        if(!looprate.sleep())
            ROS_INFO("Not enough time left");
                //looprate.sleep();
    }
    return 0;
}
