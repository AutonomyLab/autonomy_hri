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

#include <boost/thread.hpp>

#include "autonomy_human/human.h"

using namespace cv;
using namespace std;

class CHumanTracker
{
public:
	enum _trackingState{
		STATE_LOST = 0,
		STATE_DETECT = 1,
		STATE_TRACK = 2,
		STATE_REJECT = 3	
	};
	
private:
	int iWidth;
	int iHeight;
	//ros::NodeHandle node;
	Mat rawFrame;	
	Rect searchROI;
	Mat rawFramGray;
	Mat prevRawFrameGray;
	
	// Face Tracker
	KalmanFilter* KFTracker; 
	KalmanFilter* MLSearch; // Maximum Likelihood Search for multiple faces
	Mat state; // x,y,xdot,ydot,w,h
	Mat measurement;
	
	// State Machine	
	int stateCounter;
	int initialScoreMin;
	int minDetectFrames;
	int minRejectFrames;
	float maxRejectCov;
	Size minFaceSize;
	Size maxFaceSize;

	// Histograms (Skin)
	MatND faceHist;
	int hbins;
	int sbins;
	Mat skin;
	Mat skinPrior;
	Mat flow;
	Mat flowMag;
	
	double dt; // Loop length in seconds
	
	string strStates[4];
	
	/*
	 * Bit 0: Original Image
	 * Bit 1: Face Image
	 * Bit 2: Skin Image
     * Bit 3: Histogram
	 * Bit 4: Optical Flow
	 */
	unsigned short int debugLevel;
	bool skinEnabled;
	CvHaarClassifierCascade* cascade; // C API is needed for score
	
	void copyKalman(KalmanFilter* src, KalmanFilter* dest);
	void resetKalmanFilter();
	void initMats();
	void resetMats();
	void generateRegionHistogram(Mat& region, MatND &hist, bool vis = true);
	void histFilter(Mat& region, Mat& post, Mat& prior, MatND& hist,  bool vis = true);
	void detectAndTrackFace();
    void trackSkin();
	void calcOpticalFlow();
	void draw();
    
    // Threading
    boost::thread displayThread;
	
public:
	CHumanTracker(string &cascadeFile, int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, bool _skinEnabled, unsigned short int _debugLevel);
	~CHumanTracker();
	void visionCallback(const sensor_msgs::ImageConstPtr& frame);
	void reset();
	
    bool isInited;
    Mat debugFrame;
	Mat skinFrame;
	Mat opticalFrame;
	
	vector<Rect> faces;
	int faceScore;
	Rect beleif;
	_trackingState trackingState;
	bool isFaceInCurrentFrame;
};

CHumanTracker::CHumanTracker(string &cascadeFile, int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, bool _skinEnabled, unsigned short int _debugLevel)
{
	isInited = false;
	
	stateCounter = 0;
	
	skinEnabled = _skinEnabled;
	debugLevel = _debugLevel;	
	initialScoreMin = _initialScoreMin;
	minDetectFrames = _initialDetectFrames;
	minRejectFrames = _initialRejectFrames;
	
	maxRejectCov = 6.0;
	
	hbins = 15;
	sbins = 16;
		
	strStates[0] = "NOFACE";
	strStates[1] = "DETECT";
	strStates[2] = "TRACKG";
	strStates[3] = "REJECT";
	
	minFaceSize = Size(20, 25);
	maxFaceSize = Size(100,100);
	
	cascade = (CvHaarClassifierCascade*) cvLoad(cascadeFile.c_str(), 0, 0, 0);
	if (cascade == NULL)
	{
		ROS_ERROR("Problem loading cascade file %s", cascadeFile.c_str());
	}
	
	KFTracker = new KalmanFilter(6, 4, 0);
	MLSearch = new KalmanFilter(6, 4, 0);
    
}

CHumanTracker::~CHumanTracker()
{
	delete KFTracker;
	delete MLSearch;
}

void CHumanTracker::copyKalman(KalmanFilter* src, KalmanFilter* dest)
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

void CHumanTracker::reset()
{
	trackingState = STATE_LOST;
	searchROI = Rect(Point(0,0), Point(iWidth,iHeight));
	
	// x,y,xdot,ydot,w,h
	state = Mat::zeros(6, 1, CV_32F); 
	
	// x,y,w,h
	measurement = Mat::zeros(4, 1, CV_32F);
	
	initMats();
	resetMats();
	resetKalmanFilter();
}

void CHumanTracker::initMats()
{
	skin = Mat::zeros(iHeight, iWidth, CV_32F);
	skinPrior = Mat::zeros(iHeight, iWidth, CV_32F);
	faceHist = Mat::zeros(hbins, sbins, CV_32F);
    skinFrame = Mat::zeros(iHeight, iWidth, CV_8UC1);
	opticalFrame = Mat::zeros(iHeight, iWidth, CV_8UC3);
	flow = Mat::zeros(iHeight, iWidth, CV_32FC2);
	flowMag = Mat::zeros(iHeight, iWidth, CV_32F);
}

void CHumanTracker::resetMats()
{
	skinPrior = Scalar::all( 1.0 / (iWidth * iHeight)); 
	skin = Scalar::all(1.0 / (skin.rows * skin.cols));
	faceHist = Scalar::all(1.0 / (faceHist.rows * faceHist.cols) );
}
void CHumanTracker::resetKalmanFilter()
{
	
	dt = 50 * 1e-3; // This will dynamically updated.
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
			
	
}

void CHumanTracker::generateRegionHistogram(Mat& region, MatND &hist, bool vis)
{
	MatND prior = hist.clone();
	
	Mat hsv;
	cvtColor(region, hsv, CV_BGR2HSV);
	
	Mat mask = Mat::zeros(region.rows, region.cols, CV_8UC1);
	for (int r = 0; r < region.rows; r++)
	{
		for (int c = 0; c < region.cols; c++)
		{
			unsigned int v = hsv.at<Vec3b>(r,c)[2];
			// TODO: Make me parameters
			if (( v > 50) && (v < 150))
			{
				mask.at<uchar>(r,c) = 1;
			}
		}
	}
	
//	namedWindow( "Face Mask", 1 );
//	imshow( "Face Mask", mask * 255 );
	
	int histSize[] = {hbins, sbins};
	float hranges[] = {0, 180};
	float sranges[] = {0, 256};
	const float* ranges[] = { hranges, sranges};	
	int channels[] = {0, 1};
	calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);
	
	for( int h = 0; h < hbins; h++ )
	{
		for( int s = 0; s < sbins; s++ )
		{		
			hist.at<float>(h,s) = (0.99 * prior.at<float>(h,s)) + (0.01 * hist.at<float>(h,s));
		}
	}
	
	// We should make it a probability dist.
	normalize(hist, hist, 1.0, 0.0, NORM_L1);
	
	if (vis)
	{
		// For vis
		Mat visHist;
		normalize(hist, visHist, 255.0, 0.00, NORM_MINMAX);

		int scale = 10;
		Mat histImg = Mat::zeros(sbins * scale, hbins* scale, CV_8UC3);
		for( int h = 0; h < hbins; h++ )
		{
			for( int s = 0; s < sbins; s++ )
			{
				float binVal = visHist.at<float>(h, s);
				int intensity = cvRound(binVal);///maxVal);
				rectangle( histImg, Point(h*scale, s*scale),
							Point( (h+1)*scale - 1, (s+1)*scale - 1),
							Scalar::all(intensity),
							CV_FILLED );
			}
		}

        if ((debugLevel & 0x08) == 0x08)
        {
            namedWindow( "H-S Histogram", 1 );
            imshow( "H-S Histogram", histImg );
            waitKey(1);
        }
	}
}

void CHumanTracker::histFilter(Mat& region, Mat& post, Mat& prior, MatND& hist,  bool vis)
{
	int _w = region.cols;
	int _h = region.rows;
	Mat image;
	cvtColor(region, image, CV_BGR2HSV);
	for (int r = 0; r < _h; r++ )
	{
		for (int c = 0; c < _w; c++)
		{
			int hue_bin = cvFloor(image.at<cv::Vec3b>(r,c)[0] / (180.0 / (float) hbins) );
			int sat_bin = cvFloor(image.at<cv::Vec3b>(r,c)[1] / (256.0 / (float) sbins) );
			
			float z = hist.at<float>(hue_bin, sat_bin);
			post.at<float>(r,c) = (0.7 * z) + (0.3 * prior.at<float>(r, c));
			//post.at<float>(r,c) = z * prior.at<float>(r,c);
		}
	}
	
	if (vis)
	{
		// For vis;
		Mat visPost;
		normalize(post, visPost, 1.0, 0.0, NORM_MINMAX);
		namedWindow( "Skin", 1 );
		imshow( "Skin", visPost);
	}
	//prior = post.clone();
}

void CHumanTracker::detectAndTrackFace()
{
	// Do ROI 	
	debugFrame = rawFrame.clone();
	Mat img =  this->rawFrame(searchROI);
	
    double t = 0;
	faces.clear();
	ostringstream txtstr;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)} ;
    Mat gray;
    Mat frame( cvRound(img.rows), cvRound(img.cols), CV_8UC1 );
    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
    //equalizeHist( frame, frame );

    t = (double) getTickCount();
		
	// We need C API to get score
	MemStorage storage(cvCreateMemStorage(0));
	CvMat _image = frame;
	CvSeq* _objects = cvHaarDetectObjects(&_image, cascade, storage, 
			1.2, initialScoreMin, CV_HAAR_DO_CANNY_PRUNING|CV_HAAR_SCALE_IMAGE, minFaceSize, maxFaceSize);
	
	vector<CvAvgComp> vecAvgComp;
	Seq<CvAvgComp>(_objects).copyTo(vecAvgComp);
	// End of using C API
	
	isFaceInCurrentFrame = (vecAvgComp.size() > 0);
	
	if (trackingState == STATE_LOST)
	{
		if (isFaceInCurrentFrame)
		{
			stateCounter++;
			trackingState = STATE_DETECT;
		}
	}
	
	if (trackingState == STATE_DETECT)
	{
		if (isFaceInCurrentFrame)
		{
			stateCounter++;			
		}
		else
		{
			stateCounter = 0;
			trackingState = STATE_LOST;
		}
		
		if (stateCounter > minDetectFrames)
		{
			stateCounter = 0;
			trackingState = STATE_TRACK;
		}

	}
	
	if (trackingState == STATE_TRACK)
	{
		if (!isFaceInCurrentFrame)
		{
			trackingState = STATE_REJECT;
		}
	}
	
	if (trackingState == STATE_REJECT)
	{
		float covNorm = sqrt(
						pow(KFTracker->errorCovPost.at<float>(0,0), 2) +
						pow(KFTracker->errorCovPost.at<float>(1,1), 2) 
						);
		
		if (!isFaceInCurrentFrame)
		{
			stateCounter++;
		}
		else
		{
			stateCounter = 0;
			trackingState = STATE_TRACK;
		}
		
		if ((stateCounter > minRejectFrames) && (covNorm > maxRejectCov))
		{
			trackingState = STATE_LOST;
			stateCounter = 0;
			resetKalmanFilter();	
            reset();
		}
	}
		
	if ((trackingState == STATE_TRACK) || (trackingState == STATE_REJECT))
	{
		bool updateFaceHist = false;
		// This is important:
		KFTracker->transitionMatrix.at<float>(0,2) = dt;
		KFTracker->transitionMatrix.at<float>(1,3) = dt;

		Mat pred = KFTracker->predict();      

		if (isFaceInCurrentFrame)
		{
			CvAvgComp MLFace;
			float minCovNorm = 1e24;
			int i = 0;
			for( vector<CvAvgComp>::const_iterator rr = vecAvgComp.begin(); rr != vecAvgComp.end(); rr++, i++ )
			{
				copyKalman(KFTracker, MLSearch);  
				CvRect r = rr->rect;
				r.x += searchROI.x;
				r.y += searchROI.y;
				double nr = rr->neighbors;
				Point center;
				Scalar color = colors[i%8];

				float normFaceScore = 1.0 - (nr / 40.0);
				if (normFaceScore > 1.0) normFaceScore = 1.0;
				if (normFaceScore < 0.0) normFaceScore = 0.0;
				setIdentity(MLSearch->measurementNoiseCov, Scalar_<float>::all(normFaceScore));

				center.x = cvRound(r.x + r.width*0.5);
				center.y = cvRound(r.y + r.height*0.5);
				
				measurement.at<float>(0) = r.x;
				measurement.at<float>(1) = r.y;
				measurement.at<float>(2) = r.width;
				measurement.at<float>(3) = r.height;
				
				MLSearch->correct(measurement);
				
				float covNorm = sqrt(
					pow(MLSearch->errorCovPost.at<float>(0,0), 2) +
					pow(MLSearch->errorCovPost.at<float>(1,1), 2) 
				);

				if (covNorm < minCovNorm) 
				{
					minCovNorm = covNorm;
					MLFace = *rr;
				}
				
                if ((debugLevel & 0x02) == 0x02)
                {
                    rectangle(debugFrame, center - Point(r.width*0.5, r.height*0.5), center + Point(r.width*0.5, r.height * 0.5), color);

                    txtstr.str("");
                    txtstr << "   N:" << rr->neighbors << " S:" << r.width << "x" << r.height;

                    putText(debugFrame, txtstr.str(), center, FONT_HERSHEY_PLAIN, 1, color);
                }
			}

			// TODO: I'll fix this shit
			Rect r = MLFace.rect;
			r.x += searchROI.x;
			r.y += searchROI.y;
			double nr = MLFace.neighbors;
			faceScore = nr;
			float normFaceScore = 1.0 - (nr / 40.0);
			if (normFaceScore > 1.0) normFaceScore = 1.0;
			if (normFaceScore < 0.0) normFaceScore = 0.0;
			setIdentity(KFTracker->measurementNoiseCov, Scalar_<float>::all(normFaceScore));
			measurement.at<float>(0) = r.x;
			measurement.at<float>(1) = r.y;
			measurement.at<float>(2) = r.width;
			measurement.at<float>(3) = r.height;
			KFTracker->correct(measurement);
            
            // We see a face
			updateFaceHist = true;
		}
		else
		{
			KFTracker->statePost = KFTracker->statePre;
			KFTracker->errorCovPost = KFTracker->errorCovPre;
		}
		
		beleif.x = max<int>(KFTracker->statePost.at<float>(0), 0);
		beleif.y = max<int>(KFTracker->statePost.at<float>(1), 0);
		beleif.width = min<int>(KFTracker->statePost.at<float>(4), iWidth - beleif.x);
		beleif.height = min<int>(KFTracker->statePost.at<float>(5), iHeight - beleif.y);
		
		Point belCenter;
		belCenter.x = beleif.x + (beleif.width * 0.5);
		belCenter.y = beleif.y + (beleif.height * 0.5);

		double belRad = sqrt(pow(beleif.width,2) + pow(beleif.height,2)) * 0.5;

//		double faceUnc = norm(KFTracker->errorCovPost, NORM_L2);
		double faceUncPos = sqrt(
				pow(KFTracker->errorCovPost.at<float>(0,0), 2) +
				pow(KFTracker->errorCovPost.at<float>(1,1), 2) +
				pow(KFTracker->errorCovPost.at<float>(4,4), 2) +
				pow(KFTracker->errorCovPost.at<float>(5,5), 2) 
				);
		
        if ((debugLevel & 0x02) == 0x02)
        {
            txtstr.str("");
            txtstr << "P:" << std::setprecision(3) << faceUncPos << " S:" << beleif.width << "x" << beleif.height;
            putText(debugFrame, txtstr.str(), belCenter + Point(0, 50), FONT_HERSHEY_PLAIN, 2, CV_RGB(255,0,0));

            circle(debugFrame, belCenter, belRad, CV_RGB(255,0,0));
            circle(debugFrame, belCenter, (belRad - faceUncPos < 0) ? 0 : (belRad - faceUncPos), CV_RGB(255,255,0));
            circle(debugFrame, belCenter, belRad + faceUncPos, CV_RGB(255,0,255));
        }
			
		searchROI.x = max<int>(belCenter.x - KFTracker->statePost.at<float>(4) * 2, 0);
		searchROI.y = max<int>(belCenter.y - KFTracker->statePost.at<float>(5) * 2, 0);
        int x2 = min<int>(belCenter.x + KFTracker->statePost.at<float>(4) * 2, iWidth);
        int y2 = min<int>(belCenter.y + KFTracker->statePost.at<float>(4) * 2, iHeight);
		searchROI.width = x2 - searchROI.x;
		searchROI.height = y2 - searchROI.y;
	
        
		if ((updateFaceHist) && (skinEnabled))
		{
            //updateFaceHist is true when we see a real face (not all the times)
            Rect samplingWindow;
//            samplingWindow.x = beleif.x + (0.25 * beleif.width);
//            samplingWindow.y = beleif.y + (0.1 * beleif.height);
//            samplingWindow.width = beleif.width * 0.5;
//            samplingWindow.height = beleif.height * 0.9;
            samplingWindow.x = measurement.at<float>(0) + (0.25 * measurement.at<float>(2));
            samplingWindow.y = measurement.at<float>(1) + (0.10 * measurement.at<float>(3));
            samplingWindow.width = measurement.at<float>(2) * 0.5;
            samplingWindow.height = measurement.at<float>(3) * 0.9;
            if ((debugLevel & 0x04) == 0x04)
            {
                rectangle(debugFrame, samplingWindow, CV_RGB(255,0,0));
            }
			Mat _face = rawFrame(samplingWindow);
			generateRegionHistogram(_face, faceHist);
		}
		
		
	}

    if ((debugLevel & 0x02) == 0x02)
    {
        rectangle(debugFrame, searchROI, CV_RGB(0,0,0));
        txtstr.str("");
        txtstr << strStates[trackingState] << "(" << std::setprecision(3) << (dt * 1e3) << "ms )";
        putText(debugFrame, txtstr.str() , Point(30,300), FONT_HERSHEY_PLAIN, 2, CV_RGB(255,255,255));
    }
	
	dt =  ((double) getTickCount() - t) / ((double) getTickFrequency()); // In Seconds	
}

void CHumanTracker::trackSkin()
{
    
    bool perform = 
        (skinEnabled) &&
        (
            (trackingState == STATE_TRACK) || 
            (trackingState == STATE_REJECT)
        );
    
    if (perform)
    {
        histFilter(rawFrame, skin, skinPrior, faceHist, false);
        skinPrior = skin.clone();
        if ((debugLevel & 0x04) == 0x04)
        {
            Mat visSkin;
            normalize(skin, visSkin, 1.0, 0.0, NORM_MINMAX);
            visSkin.convertTo(skinFrame, CV_8UC1, 255.0, 0.0);
        }
    }
}

void CHumanTracker::calcOpticalFlow()
{
	static bool first = true;
	
	if (first)
	{
		first = false;
		cvtColor(rawFrame, prevRawFrameGray, CV_BGR2GRAY);
		return;
	}
	
	cvtColor(rawFrame, rawFramGray, CV_BGR2GRAY);
	
	bool cancelCameraMovement = (trackingState == STATE_TRACK);	

	// TODO: Optimization here
	calcOpticalFlowFarneback(prevRawFrameGray, rawFramGray, flow, 0.5, 3, 5, 3, 5, 1.1, 
	OPTFLOW_USE_INITIAL_FLOW);		
	std::vector<Mat> flowChannels;
	split(flow, flowChannels);
	magnitude(flowChannels[0], flowChannels[1], flowMag);
	threshold(flowMag, flowMag, 6, 0.0, THRESH_TOZERO);
	normalize(flowMag, flowMag, 0.0, 1.0, NORM_MINMAX);
	//threshold(flowMag, flowMag, 0.25, 0.0, THRESH_TOZERO);
	
//	Point2f biasInFlow(0.0, 0.0);
//	if (false) //(cancelCameraMovement)
//	{
//		Rect samplingWindow;
//		samplingWindow.x = measurement.at<float>(0) + (0.10 * measurement.at<float>(2));
//		samplingWindow.y = measurement.at<float>(1) + (0.10 * measurement.at<float>(3));
//		samplingWindow.width = measurement.at<float>(2) * 0.9;
//		samplingWindow.height = measurement.at<float>(3) * 0.9;
//		Mat windowFlow = flow(samplingWindow);
//		for (int y = 0; y < samplingWindow.height; y++)
//		{
//			for (int x = 0; x < samplingWindow.width; x++)
//			{
//				biasInFlow += windowFlow.at<Point2f>(y,x);
//			}
//		}
//		
//		biasInFlow.x = biasInFlow.x / (samplingWindow.height * samplingWindow.width);
//		biasInFlow.y = biasInFlow.y / (samplingWindow.height * samplingWindow.width);
//		ROS_INFO("Bias is %4.2f %4.2f", biasInFlow.x, biasInFlow.y);
//	}
//	
//	int step = 1;
//	for(int y = 0; y < opticalFrame.rows; y += step)
//	{
//		for(int x = 0; x < opticalFrame.cols; x += step)
//		{
//			Point2f fxy = flow.at<Point2f>(y, x);
//
//			// Hack
//			if ( (fabs(fxy.x) > 0.01) && (fabs(fxy.y) > 0.01))
//			{
//				fxy -= biasInFlow;
//			}
//		}
//	}


	std::swap(prevRawFrameGray, rawFramGray);
	
	// Visulization	
	if ((debugLevel & 0x10) == 0x10)
	{
		std::vector<Mat> channels;		
		split(rawFrame, channels);
		
		// HSV Color Space
		// Red to blue specterum is between 0->120 degrees (Red:0, Blue:120)
		flowMag.convertTo(channels[0], CV_8UC1, 120);
		channels[0] = Scalar::all(120) - channels[0];
		//flowMag = 120 - flowMag;
		channels[1] = Scalar::all(255.0);
		channels[2] = rawFramGray;//Scalar::all(255.0);
//		int step = 1;
//		for(int y = 0; y < opticalFrame.rows; y += step)
//		{
//			for(int x = 0; x < opticalFrame.cols; x += step)
//			{
//				Point2f fxy = flow.at<Point2f>(y, x);
//				
//				// Hack
//				if ( (fabs(fxy.x) > 0.01) && (fabs(fxy.y) > 0.01))
//				{
//					fxy -= biasInFlow;
//				}
//				
////				float rr = min<float>(255.0, (fxy.x / 25.0) * 255.0);
////				float gg = min<float>(255.0, (fxy.y / 25.0) * 255.0);
////				
////				channels[1].at<unsigned char>(y,x) = (unsigned char) gg;
////				channels[2].at<unsigned char>(y,x) = (unsigned char) rr;
//				
//				 
//				// Red to blue specterum is between 0->120 degrees (Red:0, Blue:120)				
//				float ff = 120.0 * sqrt(pow(fxy.x, 2.0) + pow(fxy.y, 2.0));
//				channels[0].at<unsigned char>(y,x) = (unsigned char) ff;
//			}
//		}
//		normalize(channels[0], channels[0], 0.0, 120.0, NORM_MINMAX);
//		threshold(channels[0], channels[0], 40, 0, THRESH_TOZERO);
		merge(channels, opticalFrame);
		cvtColor(opticalFrame, opticalFrame, CV_HSV2BGR);
	}
	
}

void CHumanTracker::draw()
{
    return ;
    
//    static bool firstTime = true;
//    
//	if ((debugLevel & 0x01) == 0x01)
//	{
//		if (firstTime) namedWindow("Input Image", 1);
//		imshow("Input Image", rawFrame);
//	}
//	
//	if ((debugLevel & 0x02) == 0x02)
//	{
//		if (firstTime) namedWindow("Face Image", 1);
//		imshow("Face Image", debugFrame);
//	}
//	
//	if ((skinEnabled) && ((debugLevel & 0x04) == 0x04))
//	{
//		if (firstTime) namedWindow("Skin Image", 1);
//		imshow("Skin Image", skinFrame);
//	}
//	
//    if (firstTime) firstTime = false;
//	waitKey(1);
}

void CHumanTracker::visionCallback(const sensor_msgs::ImageConstPtr& frame)
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
        ROS_INFO("Image size is %d x %d", iWidth, iHeight);
		reset();
	}
	
	//TODO: Check clone
	this->rawFrame = cv_ptr->image;

	detectAndTrackFace();
    trackSkin();
	calcOpticalFlow();
//	if (debugLevel > 0) draw();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "autonomy_human");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
		
	//TODO: Get from rosparam
	string xmlFile = "../cascades/haarcascade_frontalface_default.xml";
    
    //CHumanTracker* humanTracker = new CHumanTracker(xmlFile, 5, 6, 6, true, 0x06);
    
    std::string paramName;
    
    paramName.assign("~skin_enabled");
    bool skinEnabled;
    if (false == ros::param::get( paramName, skinEnabled))
        skinEnabled = false;
    
    paramName.assign("~debug_mode");
    int  debugMode;
    if (false == ros::param::get( paramName, debugMode))
        debugMode = 0x02;
    
	CHumanTracker* humanTracker = new CHumanTracker(xmlFile, 5, 6, 6, skinEnabled, debugMode);
	
	/**
	 * The queue size seems to be very important in this project
	 * If we can not complete the calculation in the 1/fps time slot, we either 
	 * drop the packet (low queue size) or process all frames which will lead to
	 * additive delays.
	 */ 
	
	image_transport::Subscriber visionSub = it.subscribe("input_rgb_image", 1, &CHumanTracker::visionCallback, humanTracker);
	ros::Publisher facePub = n.advertise<autonomy_human::human>("human", 5);  
    image_transport::Publisher debugPub = it.advertise("output_rgb_debug", 1);
    image_transport::Publisher skinPub = it.advertise("output_rgb_skin", 1);
	image_transport::Publisher opticalPub = it.advertise("output_rgb_optical", 1);

	
	ROS_INFO("Starting Autonomy Human ...");
	
	ros::Rate loopRate(50);
	autonomy_human::human msg;
    
    cv_bridge::CvImage cvi;
    sensor_msgs::Image im;
    cvi.header.frame_id = "image";
    
	while (ros::ok()){
		
		if (
				(humanTracker->trackingState == CHumanTracker::STATE_TRACK) ||
				(humanTracker->trackingState == CHumanTracker::STATE_REJECT)
			)
		{
			msg.numFaces = humanTracker->faces.size();
			msg.faceScore = humanTracker->faceScore;
			msg.faceROI.x_offset = humanTracker->beleif.x;
			msg.faceROI.y_offset = humanTracker->beleif.y;
			msg.faceROI.width = humanTracker->beleif.width;
			msg.faceROI.height = humanTracker->beleif.height;
			facePub.publish(msg);
		}
        
        if (
//                (debugPub.getNumSubscribers() > 0) && 
                ((debugMode & 0x02) == 0x02) &&
                (humanTracker->isInited)
           )
        {
            cvi.header.stamp = ros::Time::now();
            cvi.encoding = "bgr8";
            cvi.image = humanTracker->debugFrame;
            cvi.toImageMsg(im);
            debugPub.publish(im);
        }
        
        if (
//                (skinPub.getNumSubscribers() > 0) && 
                ((debugMode & 0x04) == 0x04) &&
                (humanTracker->isInited) &&
                (skinEnabled)
           )
        {
            cvi.header.stamp = ros::Time::now();
            cvi.encoding = "mono8";
            cvi.image = humanTracker->skinFrame;
            cvi.toImageMsg(im);
            skinPub.publish(im);
        }
		
		if (
                ((debugMode & 0x10) == 0x10) &&
                (humanTracker->isInited)
           )
        {
            cvi.header.stamp = ros::Time::now();
            cvi.encoding = "bgr8";
            cvi.image = humanTracker->opticalFrame;
            cvi.toImageMsg(im);
            opticalPub.publish(im);
        }
		
        
		ros::spinOnce();
		loopRate.sleep();
	}
	
	delete humanTracker;
	return 0;
}

