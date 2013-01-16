#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Header.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;

bool shouldPublish = false;
bool isInited = false;
unsigned int iWidth, iHeight;
Mat rawFrame;
Mat debugFrame;

ros::Time tStart, tEnd;

void calcOpticalFlow()
{
    flip(rawFrame, debugFrame, 0);
}

void visionCallback(const sensor_msgs::ImageConstPtr& frame)
{
    tStart = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        shouldPublish = true;
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Error converting the input image: %s", e.what());
        return;
    }

    if (isInited == false)
    {
        // Get Image Info from the first frame
        isInited = true;
        iWidth = cv_ptr->image.cols;
        iHeight = cv_ptr->image.rows;
        ROS_INFO("Image size is %d x %d", iWidth, iHeight);
    }

    //TODO: Check clone
    rawFrame = cv_ptr->image;

    calcOpticalFlow();

    tEnd = ros::Time::now();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_flow");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);


    image_transport::Subscriber visionSub = it.subscribe("input_rgb_image", 1, visionCallback);
    image_transport::Publisher debugPub = it.advertise("output_rgb_debug", 1);


    ROS_INFO("Starting Test Flow ...");

    ros::Rate loopRate(30);

    cv_bridge::CvImage cvi;
    sensor_msgs::Image im;
    cvi.header.frame_id = "image";

    while (ros::ok()){

        if (shouldPublish) {
            cvi.header.stamp = ros::Time::now();
            cvi.encoding = "bgr8";
            cvi.image = debugFrame;
            cvi.toImageMsg(im);
            debugPub.publish(im);
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
