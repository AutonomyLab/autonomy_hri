#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "pi_tracker/Skeleton.h"
#include "geometry_msgs/Vector3.h"
#include <vector>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "hark_msgs/HarkSource.h"
#include "hark_msgs/HarkSourceVal.h"
#include "sensor_msgs/LaserScan.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#define _USE_MATH_DEFINES
#define _GRID_ANGLE_MIN -90.0
#define _GRID_ANGLE_MAX 90.0
#define _GRID_ANGLE_RESOLUTION 10.0

#define _GRID_RANGE_MIN 0.0
#define _GRID_RANGE_MAX 8.0
#define _GRID_RANGE_RESOLUTION 0.5

#define _PIXEL_RESOLUTION 100

#define _SHOW_GRIDS true

using namespace cv;

const unsigned int GRID_COLS = (_GRID_ANGLE_MAX - _GRID_ANGLE_MIN)/_GRID_ANGLE_RESOLUTION;
const unsigned int GRID_ROWS = (_GRID_RANGE_MAX - _GRID_RANGE_MIN)/_GRID_RANGE_RESOLUTION;
const unsigned int GRID_SIZE = GRID_COLS*GRID_ROWS;

Mat laser_grid_img, skel_grid_img, hark_grid_img, human_grid_img;

/*Geometry_msgs/PoseArray.msg
 * std_msgs/Header header
 * geometry_msgs/Pose[] poses
 *      geometry_msgs/Pose.msg
 *          geometry_msgs/Point position (in ros coordinate frame convention)
 *          geometry_msgs/Quaternion orientation
 * /*/

typedef struct{
    float rangeMin, rangeMax, rangeRes, angleMin, angleMax, angleRes;
    //float data[GRID_ROWS][GRID_COLS];// The likelihood [0..100] for convinience
    float** data;
}polarGrid_t;

polarGrid_t* laserGrid;
polarGrid_t* userGrid;
polarGrid_t* harkGrid;
polarGrid_t* humanGrid;


void show_grid(const std::string& imgname, Mat img)
{
    imshow(imgname,img);
    waitKey(1);
}

void base_gridlines(Mat img, float h = _GRID_RANGE_MAX*_PIXEL_RESOLUTION, float w = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    for(unsigned int i = 0; i <= _GRID_RANGE_MAX*2; i++)
        circle(img, Point(w/2,h),(i*_GRID_RANGE_RESOLUTION)*_PIXEL_RESOLUTION,Scalar(255));
    for(unsigned int i = 1; i < 18; i ++)
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

void set_grid(float **array,
            const unsigned int nrows,
            const unsigned int ncolumns,
            const float val)
{
for(unsigned int i = 0; i < nrows; i++){
    for(unsigned int j = 0; j < ncolumns; j++)
        array[i][j] = val;
    }
}

void fill_cell(Mat &img,
               const float rr,
               const float tt,
               float score = 1,
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

void fill_grid(Mat &img, polarGrid_t* grid)
{
    for(unsigned int rr = 0; rr < GRID_ROWS; rr++){
        for(unsigned int cc = 0; cc < GRID_COLS; cc++){

            float r = (rr + 0.5)*_GRID_RANGE_RESOLUTION;
            float t = (cc + 0.5)*_GRID_ANGLE_RESOLUTION;
            //ROS_INFO("[%.2f] away and in [%.2f] degree",r,t);
            fill_cell(img, r,t,grid->data[rr][cc]);
        }
    }
}
Mat create_img_grid(const float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION, const float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION)
{
    //float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    //float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    Mat img = Mat::zeros(height,width,CV_8UC1);
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

polarGrid_t* create_grid(const float rmin,
                         const float rmax,
                         const float rres,
                         const float tmin,
                         const float tmax,
                         const float tres,
                         const float val)
{
    polarGrid_t* grid = (polarGrid_t*) malloc(sizeof(polarGrid_t));
    assert(grid != 0);

    grid->data = (float**) malloc(sizeof(float*)*GRID_ROWS);
    assert(grid->data != 0);
    for(unsigned int i = 0; i < GRID_ROWS; i++){
        grid->data[i] = (float*) malloc(sizeof(float)*GRID_COLS);
        assert(grid->data[i] != 0);
    }

    set_grid(grid->data,GRID_ROWS,GRID_COLS,0.0);
    grid->angleMax = tmax;
    grid->angleMin = tmin;
    grid->angleRes = tres;
    grid->rangeRes = rres;
    grid->rangeMax = rmax;
    grid->rangeMin = rmin;
    return grid;
}

void remove_grid(polarGrid_t* grid)
{
    for(unsigned int i = 0; i < GRID_ROWS; i++){
        free(grid->data[i]);
    }
    free(grid->data);
    free(grid);
}

typedef struct{
    float r, t;
}pose_t;

void cart_to_polar(const float x, const float y, float &r, float &t)
{
    r = sqrt((x * x) + (y * y));
    t = atan2(y, x)*180/M_PI;
}

float normalDistribution(const float x, const float u,const float s)
{
    return((1/(s*sqrt(2*M_PI)))*exp(-pow(x-u,2)/(2*pow(s,2))));
}

void free_likelihood(polarGrid_t* lk_grid, float pFree = 0.1)
{
    for(unsigned int r = 0; r < GRID_ROWS; r++){
        for(unsigned int c = 0; c < GRID_COLS; c++){
            lk_grid->data[r][c] += pFree;
        }
    }
}

void update_likelihood(polarGrid_t* lk_grid, std::vector<pose_t> poses)
{

    assert(lk_grid->angleMax <= _GRID_ANGLE_MAX);
    assert(lk_grid->angleMin >= _GRID_ANGLE_MIN);
    assert(lk_grid->rangeMax <= _GRID_RANGE_MAX);
    assert(lk_grid->rangeMin >= _GRID_RANGE_MIN);
    assert(lk_grid->angleRes == _GRID_ANGLE_RESOLUTION); // TODO: FIX ME
    assert(lk_grid->rangeRes == _GRID_RANGE_RESOLUTION); // TODO: FIX ME

    set_grid(lk_grid->data, GRID_ROWS, GRID_COLS, 0.0);

    unsigned int rows = (lk_grid->rangeMax - lk_grid->rangeMin)/lk_grid->rangeRes;
    unsigned int cols = (lk_grid->angleMax - lk_grid->angleMin)/lk_grid->angleRes;

    const unsigned int colStart = (lk_grid->angleMin - _GRID_ANGLE_MIN)/_GRID_ANGLE_RESOLUTION;
    const unsigned int colStop = colStart + cols;


    const unsigned int rowStart = (lk_grid->rangeMin -_GRID_RANGE_MIN)/_GRID_RANGE_RESOLUTION;
    const unsigned int rowStop = rowStart + rows;

    for(unsigned int p = 0; p < poses.size(); p++){ // on number of legs

        poses.at(p).t = poses.at(p).t - _GRID_ANGLE_MIN;

        float meanDistance = poses.at(p).r - fmod( poses.at(p).r,_GRID_RANGE_RESOLUTION) + (_GRID_RANGE_RESOLUTION/2);
        float meanAngle = fabs(poses.at(p).t) - fmod(fabs(poses.at(p).t),_GRID_ANGLE_RESOLUTION) + (_GRID_ANGLE_RESOLUTION/2);
        if(poses.at(p).t < 0)   meanAngle = -meanAngle;

        ROS_INFO("Updated to [%.2f] and [%.2f]", meanDistance,meanAngle);

        for(unsigned int r = rowStart; r < rowStop; r++){
            for(unsigned int c = colStart; c < colStop; c++){

                float pr = normalDistribution(_GRID_RANGE_RESOLUTION*(r+0.5),meanDistance,sqrt(0.2));
                float pt = normalDistribution((_GRID_ANGLE_RESOLUTION*(c+0.5)/20),meanAngle/20,sqrt(0.2));
                lk_grid->data[r][c] += pr*pt;
                //if(pr*pt > 0.01)
                  //  ROS_INFO("pr: [%.2f]  pt: [%.2f]  pr*pt [%.2f]  lk:[%.3f]", pr,pt,pr*pt,lk_grid->data[r][c]);
            }
        }
    }
/*
    for(unsigned int rr = 0; rr < GRID_ROWS; rr++){
        for(unsigned int cc = 0; cc < GRID_COLS; cc++){

            if(lk_grid->data[rr][cc] > 0.01)
                ROS_INFO("Likelihood is : [%.2f] for a person in [%.2f] away and in [%.2f] degree"
                         ,lk_grid->data[rr][cc],
                         (rr + 0.5)*_GRID_RANGE_RESOLUTION,
                         (cc + 0.5)*_GRID_ANGLE_RESOLUTION);
        }
    }
    */

}

void legs_cb(const geometry_msgs::PoseArray &msg)
{
    std::vector<geometry_msgs::Pose> legs;
    std::vector<pose_t> legPoses;
    if(!msg.poses.empty()){
        legs = msg.poses;
        //legPoses.clear();
        for(unsigned int i = 0; i < legs.size(); i++){
            pose_t tempPos;
            cart_to_polar(legs.at(i).position.x, legs.at(i).position.y, tempPos.r, tempPos.t);
            legPoses.push_back(tempPos);
        }
        ROS_INFO("*** LASER ***");
        update_likelihood(laserGrid,legPoses);
        free_likelihood(laserGrid);
        fill_grid(laser_grid_img, laserGrid);
        base_gridlines(laser_grid_img);
    }
}

void user_cb(const pi_tracker::Skeleton &msg){
    std::vector<geometry_msgs::Vector3> user;
    std::vector<pose_t> userPoses;
    if(!msg.position.empty()){
        user = msg.position;
        //userPoses.clear();
        for(unsigned int i = 0; i < user.size(); i++){
            pose_t tempPos;
            if(user.at(i).z != 0){
                cart_to_polar(user.at(i).z/1000, user.at(i).x/1000, tempPos.r, tempPos.t);
                userPoses.push_back(tempPos);
            }
        }
        ROS_INFO("*** SKELETON ***");
        update_likelihood(userGrid,userPoses);
        free_likelihood(userGrid);
        fill_grid(skel_grid_img, userGrid);
        base_gridlines(skel_grid_img);
    }
}

void hark_cb(const hark_msgs::HarkSource &msg){
    std::vector<hark_msgs::HarkSourceVal> src;
    std::vector<pose_t> harkPoses;
    if(!msg.src.empty()){
        src = msg.src;
        //harkPoses.clear();

        for(unsigned int i = 0; i < src.size(); i++){
            pose_t tempPos;
            for(float j = 0; j < harkGrid->rangeMax; j += harkGrid->rangeRes ){

                tempPos.t = src.at(i).azimuth;
                tempPos.r = j + (harkGrid->rangeRes)/2;
                harkPoses.push_back(tempPos);
            }
        }
        ROS_INFO("*** SOUND ***");

        update_likelihood(harkGrid,harkPoses);
        free_likelihood(harkGrid);
        fill_grid(hark_grid_img, harkGrid);
        base_gridlines(hark_grid_img);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"likelihood_field");
    ros::NodeHandle n;
    ROS_INFO("Creating the likelihood field ...");
    ros::Rate looprate(30);

    laserGrid = create_grid(_GRID_RANGE_MIN,_GRID_RANGE_MAX,_GRID_RANGE_RESOLUTION,
                            _GRID_ANGLE_MIN,_GRID_ANGLE_MAX,_GRID_ANGLE_RESOLUTION,0.0);
    userGrid = create_grid( 0.5, 5.00, _GRID_RANGE_RESOLUTION,
                            -30.0, 30.0, _GRID_ANGLE_RESOLUTION, 0.0);
    harkGrid = create_grid(_GRID_RANGE_MIN,_GRID_RANGE_MAX,_GRID_RANGE_RESOLUTION,
                            _GRID_ANGLE_MIN,_GRID_ANGLE_MAX,_GRID_ANGLE_RESOLUTION,0.0);
    humanGrid = create_grid(_GRID_RANGE_MIN,_GRID_RANGE_MAX,_GRID_RANGE_RESOLUTION,
                            _GRID_ANGLE_MIN,_GRID_ANGLE_MAX,_GRID_ANGLE_RESOLUTION,0.0);

    ros::Subscriber legs_sub = n.subscribe("legs",10,legs_cb);
    ros::Subscriber user_sub = n.subscribe("user",10,user_cb);
    ros::Subscriber hark_sub = n.subscribe("HarkSource", 10, hark_cb);

    //float w = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    //float h = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;

    laser_grid_img = create_img_grid();
    skel_grid_img = create_img_grid();
    hark_grid_img = create_img_grid();
    human_grid_img = create_img_grid();


    while (ros::ok()) {


    float width = 2*_GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    float height = _GRID_RANGE_MAX*_PIXEL_RESOLUTION;
    for(unsigned int i = 0; i < height; i ++){
        for(unsigned int j = 0; j < width; j++){

        }
    }


        if (_SHOW_GRIDS){
            show_grid("LASER GRID VISUALIZATION",laser_grid_img);
            show_grid("SKELETON GRID VISUALIZATION",skel_grid_img);
            show_grid("HARK GRID VISUALIZATION",hark_grid_img);
            //show_grid("HUMAN GRID VISUALIZATION",human_grid_img);
        }

        ros::spinOnce();
        looprate.sleep();
    }
    remove_grid(laserGrid);
    remove_grid(userGrid);
    remove_grid(harkGrid);
    remove_grid(humanGrid);
    return 0;
}
