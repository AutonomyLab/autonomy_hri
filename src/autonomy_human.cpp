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

using namespace cv;



class CFaceTracker
{
public:
	enum _trackingState{
		STATE_LOST = 0,
		STATE_DETECT = 1,
		STATE_TRACK = 2,
		STATE_REJECT = 3	
	};
	
private:
	ros::NodeHandle node;
	Mat rawFrame;
	Rect searchROI;
	
	// Face Tracker
	KalmanFilter* KFTracker; 
	KalmanFilter* MLSearch; // Maximum Likelihood Search for multiple faces
	Mat state; // x,y,xdot,ydot,w,h
	Mat measurement;
	
	// State Machine
	_trackingState trackingState;
	int stateCounter;
	int initialScoreMin;
	int minDetectFrames;
	int minRejectFrames;
	float maxRejectCov;

	// Histograms (Skin)
	int hbins;
	int sbins;
	MatND faceHist;

	Mat skin;
	Mat skinPrior;
	Mat skinImage;
	
public:
	CFaceTracker(int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames);
	void visionCallback(const sensor_msgs::ImageConstPtr& frame);
};

CFaceTracker::CFaceTracker(int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames)
{
	stateCounter = 0;
	
	initialScoreMin = _initialScoreMin;
	minDetectFrames = _initialDetectFrames;
	minRejectFrames = _initialRejectFrames;
	
	maxRejectCov = 6.0;
	
	hbins = 15;
	sbins = 16;
}
void CFaceTracker::visionCallback(const sensor_msgs::ImageConstPtr& frame)
{
	cv_bridge::CvImagePtr cv_ptr;    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Error converting the input image: %s", e.what());
        return;
    }
	
	namedWindow("test");
	imshow("test",cv_ptr->image);
	waitKey(3);
}

int main(int argc, char **argv)
{
	return 0;
}

