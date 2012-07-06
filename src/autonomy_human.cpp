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
using namespace std;

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
	bool isInited;
	int iWidth;
	int iHeight;
	//ros::NodeHandle node;
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
	MatND faceHist;
	int hbins;
	int sbins;
	Mat skin;
	Mat skinPrior;
	Mat skinImage;
	
	string strStates[4];
	
	void copyKalman(KalmanFilter* src, KalmanFilter* dest);
	
public:
	CFaceTracker(int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames);
	void visionCallback(const sensor_msgs::ImageConstPtr& frame);
	void reset();
};

CFaceTracker::CFaceTracker(int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames)
{
	isInited = false;
	stateCounter = 0;
	
	initialScoreMin = _initialScoreMin;
	minDetectFrames = _initialDetectFrames;
	minRejectFrames = _initialRejectFrames;
	
	maxRejectCov = 6.0;
	
	hbins = 15;
	sbins = 16;
	
	// x,y,xdot,ydot,w,h
	state = Mat::zeros(6, 1, CV_32F); 
	
	// x,y,w,h
	measurement = Mat::zeros(4, 1, CV_32F);
	
	strStates[0] = "NOFACE";
	strStates[1] = "DETECT";
	strStates[2] = "TRACKG";
	strStates[3] = "REJECT";
}

void CFaceTracker::copyKalman(KalmanFilter* src, KalmanFilter* dest)
{
	dest->measurementNoiseCov = src->measurementNoiseCov.clone();
	dest->controlMatrix = src->controlMatrix.clone();
	dest->errorCovPost = src->errorCovPost.clone();
	dest->errorCovPre = src->errorCovPre.clone();
	dest->gain = src->gain.clone();
	dest->measurementMatrix = src->measurementMatrix.clone();
	dest->processNoiseCov = src->processNoiseCov.clone();
	dest->statePost = src->statePost.clone();
	dest->statePre = src->statePre.clone();
	dest->transitionMatrix = src->transitionMatrix.clone();
}

void CFaceTracker::reset()
{
	searchROI = Rect(Point(0,0), Point(iWidth,iHeight));
	measurement.setTo(Scalar(0));
	KFTracker->transitionMatrix = * (Mat_<float>(6,6) 
			<< 
			1,0,1,0,0,0,
			0,1,0,1,0,0, 
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1
			);

	KFTracker->measurementMatrix = *(Mat_<float>(4,6) 
			<< 
			1,0,0,0,0,0,
			0,1,0,0,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1
			);

	KFTracker->statePre = *(Mat_<float>(6,1) << 0,0,0,0,0,0);
	KFTracker->statePost = *(Mat_<float>(6,1) << 0,0,0,0,0,0);
	
	setIdentity(KFTracker->errorCovPre, Scalar_<float>::all(1e2));
	setIdentity(KFTracker->errorCovPost, Scalar_<float>::all(1e2));
	setIdentity(KFTracker->processNoiseCov, Scalar_<float>::all(0.05));
	setIdentity(KFTracker->measurementNoiseCov, Scalar_<float>::all(1.0));

	copyKalman(KFTracker, MLSearch);
	

	skinPrior = Scalar::all( 1.0 / (iWidth * iHeight)); 
	skin = Scalar::all(1.0 / (skin.rows * skin.cols));
	faceHist = Scalar::all(1.0 / (faceHist.rows * faceHist.cols) );
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
	
	if (isInited == false)
	{
		// Get Image Info from the first frame
		isInited = true;
		iWidth = cv_ptr->image.cols;
		iHeight = cv_ptr->image.rows;
		reset();
	}
	
	namedWindow("test");
	imshow("test",cv_ptr->image);
	waitKey(3);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "autonomy_human");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	
	CFaceTracker* faceTracker = new CFaceTracker(5, 6, 6);

	image_transport::Subscriber visionSub = it.subscribe("input_rgb_image", 100, &CFaceTracker::visionCallback, faceTracker);
	
	ROS_INFO("Starting Autonomy Human ...");
	
	ros::Rate loopRate(25);
	while (ros::ok()){
		ros::spinOnce();
		loopRate.sleep();
	}
	
	delete faceTracker;
	return 0;
}

