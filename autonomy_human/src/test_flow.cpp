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
Mat prevRawFrame;
Mat rawFrame;
Mat debugFrame;
Mat flow;
Mat prevRawFrameGray;
Mat rawFrameGray;
Mat flowMag;

ros::Time tStart, tFlow, tEnd;

void calcOpticalFlow()
{
    static bool first = true;

    if (first) {
        first = false;
        return;
    }

//    calcOpticalFlowSF(prevRawFrame, rawFrame,
//                      flow,
//                      3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);

//    calcOpticalFlowSF(prevRawFrameGray, rawFrameGray,
//                      flow,
//                      1, 2, 4);


    calcOpticalFlowFarneback( prevRawFrameGray, rawFrameGray , flow, 0.5, 3, 5, 3, 7, 1.5, 0);//OPTFLOW_USE_INITIAL_FLOW);

    std::vector<Mat> flowChannels;
    split(flow, flowChannels);
    magnitude(flowChannels[0], flowChannels[1], flowMag);
    threshold(flowMag, flowMag, 5.0, 0.0, THRESH_TOZERO);
    normalize(flowMag, flowMag, 0.0, 1.0, NORM_MINMAX);
    std::vector<Mat> channels;
    split(rawFrame, channels);
    flowMag.convertTo(channels[0], CV_8UC1, 120);
    channels[0] = Scalar::all(120) - channels[0];
    channels[1] = Scalar::all(255.0);
    channels[2] = rawFrameGray;
    merge(channels, debugFrame);
    cvtColor(debugFrame, debugFrame, CV_HSV2BGR);
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
        ROS_INFO("Image size is %d x %d | Type : %d", iWidth, iHeight, cv_ptr->image.type());
    }

    prevRawFrame = rawFrame.clone();
    prevRawFrameGray = rawFrameGray.clone();
    //TODO: Check clone
    rawFrame = cv_ptr->image;
    cvtColor(rawFrame, rawFrameGray, CV_BGR2GRAY);

    tFlow = ros::Time::now();
    calcOpticalFlow();

    tEnd = ros::Time::now();

    ROS_INFO("Flow: %d x %d x %d @ %5.3f", flow.rows, flow.cols, flow.dims, (tEnd - tFlow).toSec());
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
