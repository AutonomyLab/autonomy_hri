#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <autonomy_human/flow.h>
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
autonomy_human::flow flowMsg;


// From OpenCV fback.cpp
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, const int step, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            ROS_INFO("%4.2f %4.2f", fxy.x, fxy.y);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            //circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

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


    // TODO: nan check
    calcOpticalFlowFarneback( prevRawFrameGray, rawFrameGray , flow, 0.5, 5, 5, 3, 5, 1.1, (flow.rows != 0) ? OPTFLOW_USE_INITIAL_FLOW : 0);//OPTFLOW_USE_INITIAL_FLOW);

    std::vector<Mat> flowChannels;
    split(flow, flowChannels);
//    Mat nn;

    //bool doAvg = (flowMag.rows > 0);
    magnitude(flowChannels[0], flowChannels[1], flowMag);
    Rect roi(300,100, 100, 100);

//    if (doAvg)
//        flowMag = (0.5 * flowMag) + (0.5 * nn);
//    else
//        flowMag = nn;

    // These are the pixels that actually have a calculated flow
    Mat maskX;
    Mat maskY;
    maskX = abs(flowChannels[0]) > 0.01;
    maskY = abs(flowChannels[1]) > 0.01;
    Mat mask = maskX | maskY;

    threshold(flowMag, flowMag, 5.0 , 0.0, THRESH_TOZERO);

    flowMsg.header.stamp = ros::Time::now();

    // Note that flow_x and flow_y are signed!
    flowMsg.flow_x = sum(flowChannels[0])[0];
    flowMsg.flow_y = sum(flowChannels[1])[0];
    flowMsg.flow_mag = sum(flowMag(roi))[0];

    //normalize is WRONG when u need to make decisions based on flow
    //normalize(flowMag, flowMag, 0.0, 1.0, NORM_MINMAX);
    if (true) {
        std::vector<Mat> channels;
        split(rawFrame, channels);
        flowMag.convertTo(channels[0], CV_8UC1, 120); //120x amplification!
        channels[0] = Scalar::all(120) - channels[0];
        channels[1] = Scalar::all(255.0);
        //flowChannels[0].convertTo(channels[0], CV_8UC1, 120);
        //flowChannels[1].convertTo(channels[1], CV_8UC1, 120);
        channels[2] = rawFrameGray;
        merge(channels, debugFrame);
        cvtColor(debugFrame, debugFrame, CV_HSV2BGR);
        rectangle(debugFrame, roi, CV_RGB(255, 0, 0));
    } else {
        std::vector<Mat> channels;
        split(rawFrame, channels);
        mask.convertTo(channels[0], CV_8UC1, 120);
        channels[0] = Scalar::all(120) - channels[0];
        channels[1] = Scalar::all(255.0);
        channels[2] = rawFrameGray;
        merge(channels, debugFrame);
        cvtColor(debugFrame, debugFrame, CV_HSV2BGR);
    }

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
    ros::Publisher flowPub = n.advertise<autonomy_human::flow>("flow", 50);


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
            flowPub.publish(flowMsg);
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
