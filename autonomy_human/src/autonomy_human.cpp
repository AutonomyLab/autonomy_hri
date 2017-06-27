#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Header.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//#include <boost/thread.hpp>

#include "autonomy_human/human.h"
#include "autonomy_human/raw_detections.h"

using namespace cv;
using namespace std;

class CHumanTracker
{
public:
  enum _trackingState
  {
    STATE_LOST = 0,
    STATE_DETECT = 1,
    STATE_TRACK = 2,
    STATE_REJECT = 3,
    STATE_NUM = 4
  };

  enum _gestureRegionIds
  {
    REG_TOPLEFT = 0,
    REG_TOPRIGHT = 1,
    REG_NUM = 2
  };

  enum _stablizationMethods
  {
    STABLIZE_OFF = 0,
    STABLIZE_MEDIAN,
    STABLIZE_HOMOGRAPHY,
    STABLIZAE_NUM
  };

private:
  int iWidth;
  int iHeight;
  //ros::NodeHandle node;
  Mat rawFrame;
  Rect searchROI;
  Rect flowROI;
  Mat rawFramGray;
  Mat prevRawFrameGray;

  // Face Tracker
  KalmanFilter KFTracker;
  KalmanFilter MLSearch; // Maximum Likelihood Search for multiple faces
  Mat state; // x,y,xdot,ydot,w,h
  Mat measurement;
  float pCovScalar;
  float mCovScalar;

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

  // Optical Flow
  int minFlow;


  //Time measurements
  ros::Time tStart;
  ros::Time tFaceStart;
  ros::Time tSkinStart;
  ros::Time tFlowStart;
  ros::Time tEnd;

  string strStates[4];

  /*
   * Bit 0: Original Image
   * Bit 1: Face Image
   * Bit 2: Skin Image
     * Bit 3: Histogram
   * Bit 4: Optical Flow
   */
  unsigned short int debugLevel;
  bool profileHackEnabled;
  bool skinEnabled;
  bool gestureEnabled;

  // C API is needed for score
  // But let's add smart pointers
  cv::Ptr<CvHaarClassifierCascade> cascade;
  cv::Ptr<CvHaarClassifierCascade> cascadeProfile;

  void copyKalman(const KalmanFilter& src, KalmanFilter& dest);
  void resetKalmanFilter();
  void initMats();
  void resetMats();
  void generateRegionHistogram(Mat& region, MatND &hist, bool vis = true);
  void histFilter(Mat& region, Mat& post, Mat& prior, MatND& hist,  bool vis = true);
  void detectAndTrackFace();
  void trackSkin();
  void calcOpticalFlow();
  void draw();
  void safeRect(Rect &r, Rect &boundry);
  void safeRectToImage(Rect &r, double fx = 1.0, double fy = 1.0);

  // Temp
  CvAvgComp MLFace;
  string frame_id;

  // We need C API to get score
  // Let's use OpenCV Smart Pointers to old data structures
  cv::Ptr<CvMemStorage> storage;
  cv::Ptr<CvMemStorage> storageProfile;

  ros::Publisher& facePub;
  ros::Publisher& allDetectionsPub;
  image_transport::Publisher& debugPub;
  image_transport::Publisher& skinPub;
  image_transport::Publisher& opticalPub;

public:
  CHumanTracker(string &cascadeFile, string &cascadeFileProfile,
                float _pCov, float _mCov,
                int _minFaceSizeW, int _minFaceSizeH, int _maxFaceSizeW, int _maxFaceSizeH,
                int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, int _minFlow,
                bool _profileHackEnabled, bool _skinEnabled, bool _gestureEnabled,
                unsigned short int _debugLevel, unsigned int _stablization,
                ros::Publisher& _facePub, ros::Publisher& _allDetectionsPub,
                image_transport::Publisher& _debugPub, image_transport::Publisher& _skinPub, image_transport::Publisher& _opticalPub);

  void publish();
  ~CHumanTracker();
  void visionCallback(const sensor_msgs::ImageConstPtr& frame);
  void reset();
  float calcMedian(Mat &m, int nbins, float minVal, float maxVal, InputArray mask = noArray());

  bool isInited;
  Mat debugFrame;
  Mat skinFrame;
  Mat opticalFrame;

  vector<Rect> faces;
  int faceScore;
  Rect beleif;
  _trackingState trackingState;
  bool isFaceInCurrentFrame;

  bool shouldPublish;
  unsigned int stablization;

  /*
   * Drone Point of View, Face in center
   * 0 : Top left
   * 1 : Top right
   */
  Rect gestureRegion[2];
  float flowScoreInRegion[2];

};

//
// KFTracker = new KalmanFilter(6, 4, 0);
// MLSearch = new KalmanFilter(6, 4, 0);
CHumanTracker::CHumanTracker(string &cascadeFile, string &cascadeFileProfile,
                             float _pCov, float _mCov,
                             int _minFaceSizeW, int _minFaceSizeH, int _maxFaceSizeW, int _maxFaceSizeH,
                             int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, int _minFlow,
                             bool _profileHackEnabled, bool _skinEnabled, bool _gestureEnabled,
                             unsigned short int _debugLevel, unsigned int _stablization,
                             ros::Publisher& _facePub, ros::Publisher& _allDetectionsPub,
                             image_transport::Publisher& _debugPub, image_transport::Publisher& _skinPub, image_transport::Publisher& _opticalPub)
  : KFTracker(6, 4, 0)
  , MLSearch(6, 4, 0)
  , pCovScalar(_pCov)
  , mCovScalar(_mCov)
  , stateCounter(0)
  , initialScoreMin(_initialScoreMin)
  , minDetectFrames(_initialDetectFrames)
  , minRejectFrames(_initialRejectFrames)
  , maxRejectCov(6.0)
  , minFaceSize(_minFaceSizeW, _minFaceSizeH)
  , maxFaceSize(_maxFaceSizeW, _maxFaceSizeH)
  , hbins(15)
  , sbins(16)
  , minFlow(_minFlow)
  , debugLevel(_debugLevel)
  , profileHackEnabled(_profileHackEnabled)
  , skinEnabled(_skinEnabled)
  , gestureEnabled(_gestureEnabled)
  , storage(cvCreateMemStorage(0))
  , storageProfile(cvCreateMemStorage(0))
  , facePub(_facePub)
  , allDetectionsPub(_allDetectionsPub)
  , debugPub(_debugPub)
  , skinPub(_skinPub)
  , opticalPub(_opticalPub)
  , isInited(false)
  , trackingState(STATE_LOST)
  , shouldPublish(false)
  , stablization(_stablization)
{

  strStates[0] = "NOFACE";
  strStates[1] = "DETECT";
  strStates[2] = "TRACKG";
  strStates[3] = "REJECT";

  cascade = (CvHaarClassifierCascade*) cvLoad(cascadeFile.c_str(), 0, 0, 0);
  if (cascade.empty())
  {
    ROS_ERROR("Problem loading cascade file %s", cascadeFile.c_str());
  }

  if (profileHackEnabled)
  {
    cascadeProfile = (CvHaarClassifierCascade*) cvLoad(cascadeFileProfile.c_str(), 0, 0, 0);
    if (cascade.empty())
    {
      ROS_ERROR("Problem loading profile cascade file %s", cascadeFileProfile.c_str());
    }
  }

  isFaceInCurrentFrame = false;

}

void CHumanTracker::publish()
{
  autonomy_human::human msg;
  autonomy_human::raw_detections faces_msg;

  cv_bridge::CvImage cvi;
  sensor_msgs::Image im;
  cvi.header.frame_id = "image";

  if ((facePub.getNumSubscribers() > 0) &&
      (
        (trackingState == CHumanTracker::STATE_TRACK) ||
        (trackingState == CHumanTracker::STATE_REJECT)
      ))
  {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.numFaces = faces.size();
    msg.faceScore = faceScore;
    msg.faceROI.x_offset = beleif.x;
    msg.faceROI.y_offset = beleif.y;
    msg.faceROI.width = beleif.width;
    msg.faceROI.height = beleif.height;
    if (gestureEnabled)
    {
      for (int i = 0; i < 2; i++)
        msg.flowScore[i] = flowScoreInRegion[i];
    }
    facePub.publish(msg);
  }


  if (allDetectionsPub.getNumSubscribers() > 0)
  {
    faces_msg.header.stamp = ros::Time::now();
    faces_msg.header.frame_id = frame_id;

    for (size_t i = 0; i < faces.size(); i++)
    {
      sensor_msgs::RegionOfInterest tmp_human;
      tmp_human.x_offset = faces[i].x;
      tmp_human.y_offset = faces[i].y;
      tmp_human.width = faces[i].width;
      tmp_human.height = faces[i].height;
      faces_msg.detections.push_back(tmp_human);
    }

    allDetectionsPub.publish(faces_msg);
  }

  if (
    (debugPub.getNumSubscribers() > 0) &&
    (shouldPublish) &&
    ((debugLevel & 0x02) == 0x02) &&
    (isInited)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "bgr8";
    cvi.image = debugFrame;
    cvi.toImageMsg(im);
    debugPub.publish(im);
  }

  if (
    (skinPub.getNumSubscribers() > 0) &&
    (shouldPublish) &&
    ((debugLevel & 0x04) == 0x04) &&
    (isInited) &&
    (skinEnabled)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "mono8";
    cvi.image = skinFrame;
    cvi.toImageMsg(im);
    skinPub.publish(im);
  }

  if (
    (opticalPub.getNumSubscribers() > 0) &&
    (shouldPublish) &&
    ((debugLevel & 0x10) == 0x10) &&
    (isInited) &&
    (gestureEnabled)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "bgr8";
    cvi.image = opticalFrame;
    cvi.toImageMsg(im);
    opticalPub.publish(im);
  }
}

CHumanTracker::~CHumanTracker()
{
  ;
}

void CHumanTracker::copyKalman(const KalmanFilter& src, KalmanFilter& dest)
{
  dest.measurementNoiseCov = src.measurementNoiseCov.clone();
  dest.controlMatrix = src.controlMatrix.clone();
  dest.errorCovPost = src.errorCovPost.clone();
  dest.errorCovPre = src.errorCovPre.clone();
  dest.gain = src.gain.clone();
  dest.measurementMatrix = src.measurementMatrix.clone();
  dest.processNoiseCov = src.processNoiseCov.clone();
  dest.statePost = src.statePost.clone();
  dest.statePre = src.statePre.clone();
  dest.transitionMatrix = src.transitionMatrix.clone();
}

void CHumanTracker::reset()
{
  trackingState = STATE_LOST;
  searchROI = Rect(Point(0, 0), Point(iWidth, iHeight));

  // x,y,xdot,ydot,w,h
  state = Mat::zeros(6, 1, CV_32F);

  // x,y,w,h
  measurement = Mat::zeros(4, 1, CV_32F);

  isFaceInCurrentFrame = false;

  for (int i = 0; i < 2; i++)
    flowScoreInRegion[i] = 0.0;

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
  skinPrior = Scalar::all(1.0 / (iWidth * iHeight));
  skin = Scalar::all(1.0 / (skin.rows * skin.cols));
  faceHist = Scalar::all(1.0 / (faceHist.rows * faceHist.cols));
}
void CHumanTracker::resetKalmanFilter()
{
  measurement.setTo(Scalar(0));

  // The system model is very naive. The small effect of xdot and ydot
  // on x and y are intentional (it is 0 now)
  KFTracker.transitionMatrix = * (Mat_<float>(6, 6)
                                  <<
                                  1, 0, 0, 0, 0, 0,
                                  0, 1, 0, 0, 0, 0,
                                  0, 0, 1, 0, 0, 0,
                                  0, 0, 0, 1, 0, 0,
                                  0, 0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 0, 1
                                 );

  KFTracker.measurementMatrix = *(Mat_<float>(4, 6)
                                  <<
                                  1, 0, 0, 0, 0, 0,
                                  0, 1, 0, 0, 0, 0,
                                  0, 0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 0, 1
                                 );

  KFTracker.statePre = *(Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);
  KFTracker.statePost = *(Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);

  setIdentity(KFTracker.errorCovPre, Scalar_<float>::all(1e2));
  setIdentity(KFTracker.errorCovPost, Scalar_<float>::all(1e2));
  setIdentity(KFTracker.processNoiseCov, Scalar_<float>::all(pCovScalar));
  setIdentity(KFTracker.measurementNoiseCov, Scalar_<float>::all(mCovScalar));

  copyKalman(KFTracker, MLSearch);


}

float CHumanTracker::calcMedian(Mat &m, int nbins, float minVal, float maxVal, InputArray mask)
{
  MatND hist;
  int hsize[1];
  hsize[0] = nbins;
  float range[2];
  range[0] = minVal;
  range[1] = maxVal;
  const float *ranges[] = {range};
  int chnls[] = {0};

  calcHist(&m, 1, chnls, mask, hist, 1, hsize, ranges);

  float step = (maxVal - minVal) / nbins;
  long int sumHist = sum(hist)[0];
  long int sum = 0;
  float median = 0.0;
  for (int i = 0; i < nbins; i++)
  {
    sum += hist.at<float>(i, 0);
    median = minVal + (i * step) + (step / 2.0);
    if (sum >= (sumHist / 2.0))
    {
      break;
    }
  }
  return median;
}

void CHumanTracker::safeRect(Rect& r, Rect& boundry)
{
  r.x = max<int>(r.x, boundry.x);
  r.y = max<int>(r.y, boundry.y);

  Point2d p2;
  p2.x = min<int>(r.br().x, boundry.br().x);
  p2.y = min<int>(r.br().y, boundry.br().y);

  r.width = p2.x - r.x;
  r.height = p2.y - r.y;
}

void CHumanTracker::safeRectToImage(Rect& r, double fx, double fy)
{
  Rect safety(0, 0, iWidth * fx, iHeight * fy);
  safeRect(r, safety);
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
      unsigned int v = hsv.at<Vec3b>(r, c)[2];
      // TODO: Make me parameters
      if ((v > 10) && (v < 240))
      {
        mask.at<uchar>(r, c) = 1;
      }
    }
  }

//  namedWindow( "Face Mask", 1 );
//  imshow( "Face Mask", mask * 255 );

  int histSize[] = {hbins, sbins};
  float hranges[] = {0, 180};
  float sranges[] = {0, 256};
  const float* ranges[] = { hranges, sranges};
  int channels[] = {0, 1};
  calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);

  for (int h = 0; h < hbins; h++)
  {
    for (int s = 0; s < sbins; s++)
    {
      hist.at<float>(h, s) = (0.99 * prior.at<float>(h, s)) + (0.01 * hist.at<float>(h, s));
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
    Mat histImg = Mat::zeros(sbins * scale, hbins * scale, CV_8UC3);
    for (int h = 0; h < hbins; h++)
    {
      for (int s = 0; s < sbins; s++)
      {
        float binVal = visHist.at<float>(h, s);
        int intensity = cvRound(binVal);///maxVal);
        rectangle(histImg, Point(h * scale, s * scale),
                  Point((h + 1)*scale - 1, (s + 1)*scale - 1),
                  Scalar::all(intensity),
                  CV_FILLED);
      }
    }

    if ((debugLevel & 0x08) == 0x08)
    {
      namedWindow("H-S Histogram", 1);
      imshow("H-S Histogram", histImg);
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
  for (int r = 0; r < _h; r++)
  {
    for (int c = 0; c < _w; c++)
    {
      int hue_bin = cvFloor(image.at<cv::Vec3b>(r, c)[0] / (180.0 / (float) hbins));
      int sat_bin = cvFloor(image.at<cv::Vec3b>(r, c)[1] / (256.0 / (float) sbins));

      float z = hist.at<float>(hue_bin, sat_bin);
      post.at<float>(r, c) = (0.7 * z) + (0.3 * prior.at<float>(r, c));
      //post.at<float>(r,c) = z * prior.at<float>(r,c);
    }
  }

  if (vis)
  {
    // For vis;
    Mat visPost;
    normalize(post, visPost, 1.0, 0.0, NORM_MINMAX);
    namedWindow("Skin", 1);
    imshow("Skin", visPost);
  }
  //prior = post.clone();
}

void CHumanTracker::detectAndTrackFace()
{
  static ros::Time probe;

  // Do ROI
  debugFrame = rawFrame.clone();
  Mat img =  this->rawFrame(searchROI);

  faces.clear();
  ostringstream txtstr;
  const static Scalar colors[] =  { CV_RGB(0, 0, 255),
                                    CV_RGB(0, 128, 255),
                                    CV_RGB(0, 255, 255),
                                    CV_RGB(0, 255, 0),
                                    CV_RGB(255, 128, 0),
                                    CV_RGB(255, 255, 0),
                                    CV_RGB(255, 0, 0),
                                    CV_RGB(255, 0, 255)
                                  } ;
  Mat gray;
  Mat frame(cvRound(img.rows), cvRound(img.cols), CV_8UC1);
  cvtColor(img, gray, CV_BGR2GRAY);
  resize(gray, frame, frame.size(), 0, 0, INTER_LINEAR);
  //equalizeHist( frame, frame );

  // This if for internal usage
  const ros::Time _n = ros::Time::now();
  double dt = (_n - probe).toSec();
  probe = _n;


  CvMat _image = frame;

  if (!storage.empty())
  {
    cvClearMemStorage(storage);
  }
  CvSeq* _objects = cvHaarDetectObjects(&_image, cascade, storage,
                                        1.2, initialScoreMin, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_SCALE_IMAGE, minFaceSize, maxFaceSize);

  vector<CvAvgComp> vecAvgComp;
  Seq<CvAvgComp>(_objects).copyTo(vecAvgComp);

  // End of using C API

  isFaceInCurrentFrame = (vecAvgComp.size() > 0);

  // This is a hack
  bool isProfileFace = false;
  if ((profileHackEnabled) && (!isFaceInCurrentFrame) && ((trackingState == STATE_REJECT) || (trackingState == STATE_REJECT)))
  {
    ROS_DEBUG("Using Profile Face hack ...");

    if (!storageProfile.empty())
    {
      cvClearMemStorage(storageProfile);
    }
    CvSeq* _objectsProfile = cvHaarDetectObjects(&_image, cascadeProfile, storageProfile,
                             1.2, initialScoreMin, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_SCALE_IMAGE, minFaceSize, maxFaceSize);
    vecAvgComp.clear();
    Seq<CvAvgComp>(_objectsProfile).copyTo(vecAvgComp);
    isFaceInCurrentFrame = (vecAvgComp.size() > 0);
    if (isFaceInCurrentFrame)
    {
      ROS_DEBUG("The hack seems to work!");
    }
    isProfileFace = true;
  }

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
                      pow(KFTracker.errorCovPost.at<float>(0, 0), 2) +
                      pow(KFTracker.errorCovPost.at<float>(1, 1), 2)
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

    KFTracker.transitionMatrix.at<float>(0, 2) = dt;
    KFTracker.transitionMatrix.at<float>(1, 3) = dt;

    Mat pred = KFTracker.predict();

    if (isFaceInCurrentFrame)
    {
      //std::cout << vecAvgComp.size() << " detections in image " << std::endl;
      float minCovNorm = 1e24;
      int i = 0;
      for (vector<CvAvgComp>::const_iterator rr = vecAvgComp.begin(); rr != vecAvgComp.end(); rr++, i++)
      {
        copyKalman(KFTracker, MLSearch);
        CvRect r = rr->rect;
        r.x += searchROI.x;
        r.y += searchROI.y;
        double nr = rr->neighbors;
        Point center;
        Scalar color = colors[i % 8];

        float normFaceScore = 1.0 - (nr / 40.0);
        if (normFaceScore > 1.0) normFaceScore = 1.0;
        if (normFaceScore < 0.0) normFaceScore = 0.0;
        setIdentity(MLSearch.measurementNoiseCov, Scalar_<float>::all(normFaceScore));

        center.x = cvRound(r.x + r.width * 0.5);
        center.y = cvRound(r.y + r.height * 0.5);

        measurement.at<float>(0) = r.x;
        measurement.at<float>(1) = r.y;
        measurement.at<float>(2) = r.width;
        measurement.at<float>(3) = r.height;

        MLSearch.correct(measurement);

        float covNorm = sqrt(
                          pow(MLSearch.errorCovPost.at<float>(0, 0), 2) +
                          pow(MLSearch.errorCovPost.at<float>(1, 1), 2)
                        );

        if (covNorm < minCovNorm)
        {
          minCovNorm = covNorm;
          MLFace = *rr;
        }

//                if ((debugLevel & 0x02) == 0x02)
//                {
        rectangle(debugFrame, center - Point(r.width * 0.5, r.height * 0.5), center + Point(r.width * 0.5, r.height * 0.5), color);

        txtstr.str("");
        txtstr << "   Sc:" << rr->neighbors << " S:" << r.width << "x" << r.height;

        putText(debugFrame, txtstr.str(), center, FONT_HERSHEY_PLAIN, 1, color);
//                }
      }

      // TODO: I'll fix this shit
      Rect r(MLFace.rect);
      r.x += searchROI.x;
      r.y += searchROI.y;
      faces.push_back(r);
      double nr = MLFace.neighbors;
      faceScore = nr;
      if (isProfileFace) faceScore = 0.0;
      float normFaceScore = 1.0 - (nr / 40.0);
      if (normFaceScore > 1.0) normFaceScore = 1.0;
      if (normFaceScore < 0.0) normFaceScore = 0.0;
      setIdentity(KFTracker.measurementNoiseCov, Scalar_<float>::all(normFaceScore));
      measurement.at<float>(0) = r.x;
      measurement.at<float>(1) = r.y;
      measurement.at<float>(2) = r.width;
      measurement.at<float>(3) = r.height;
      KFTracker.correct(measurement);

      // We see a face
      updateFaceHist = true;
    }
    else
    {
      KFTracker.statePost = KFTracker.statePre;
      KFTracker.errorCovPost = KFTracker.errorCovPre;
    }

    // TODO: MOVE THIS
    for (unsigned int k = 0; k < faces.size(); k++)
    {
      rectangle(debugFrame, faces.at(k), CV_RGB(128, 128, 128));
    }

    beleif.x = max<int>(KFTracker.statePost.at<float>(0), 0);
    beleif.y = max<int>(KFTracker.statePost.at<float>(1), 0);
    beleif.width = min<int>(KFTracker.statePost.at<float>(4), iWidth - beleif.x);
    beleif.height = min<int>(KFTracker.statePost.at<float>(5), iHeight - beleif.y);

    Point belCenter;
    belCenter.x = beleif.x + (beleif.width * 0.5);
    belCenter.y = beleif.y + (beleif.height * 0.5);

    if ((debugLevel & 0x02) == 0x02)
    {
      txtstr.str("");
//            txtstr << "P:" << std::setprecision(3) << faceUncPos << " S:" << beleif.width << "x" << beleif.height;
//            putText(debugFrame, txtstr.str(), belCenter + Point(0, 50), FONT_HERSHEY_PLAIN, 2, CV_RGB(255,0,0));

//            circle(debugFrame, belCenter, belRad, CV_RGB(255,0,0));
//            circle(debugFrame, belCenter, (belRad - faceUncPos < 0) ? 0 : (belRad - faceUncPos), CV_RGB(255,255,0));
//            circle(debugFrame, belCenter, belRad + faceUncPos, CV_RGB(255,0,255));
    }

    //searchROI.x = max<int>(belCenter.x - KFTracker.statePost.at<float>(4) * 2, 0);
    //searchROI.y = max<int>(belCenter.y - KFTracker.statePost.at<float>(5) * 2, 0);
    //int x2 = min<int>(belCenter.x + KFTracker.statePost.at<float>(4) * 2, iWidth);
    //int y2 = min<int>(belCenter.y + KFTracker.statePost.at<float>(4) * 2, iHeight);
    //searchROI.width = x2 - searchROI.x;
    //searchROI.height = y2 - searchROI.y;


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
        rectangle(debugFrame, samplingWindow, CV_RGB(255, 0, 0));
      }
      Mat _face = rawFrame(samplingWindow);
      generateRegionHistogram(_face, faceHist);
    }


  }


//    if ((debugLevel & 0x02) == 0x02)
//    {
//        rectangle(debugFrame, searchROI, CV_RGB(0,0,0));
//        txtstr.str("");
//        txtstr << strStates[trackingState] << "(" << std::setprecision(3) << (dt * 1e3) << "ms )";
//        putText(debugFrame, txtstr.str() , Point(30,300), FONT_HERSHEY_PLAIN, 2, CV_RGB(255,255,255));
//    }

//  dt =  ((double) getTickCount() - t) / ((double) getTickFrequency()); // In Seconds
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
    const double sum_skin = sum(skin)[0];
    skin *= (1.0 / sum_skin);
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
  static bool firstCancel = true;
  static Rect prevFaceRect;

  bool perform =
    (gestureEnabled) &&
    (
      (trackingState == STATE_TRACK) ||
      (trackingState == STATE_REJECT)
    );

  if (!perform) return;

  Mat rawFrameResized;
  const double fx = 0.5;
  const double fy = 0.5;
  resize(rawFrame, rawFrameResized, Size(), fx, fy,  INTER_LINEAR);

  if (first)
  {
    first = false;
    cvtColor(rawFrameResized, prevRawFrameGray, CV_BGR2GRAY);
    return;
  }

  Point belCenter;
  belCenter.x = (fx * beleif.x) + (fx * beleif.width * 0.5);
  belCenter.y = (fy * beleif.y) + (fy * beleif.height * 0.5);

  //‌ FlowRoi is now not being used to filter out any region, it is a placeholder
  // changed by Jake
  flowROI.x = max<int>(belCenter.x - fx * KFTracker.statePost.at<float>(4) / 1.5, 0);
  flowROI.y = max<int>(belCenter.y - fy * KFTracker.statePost.at<float>(5), 0);
  int x2 = min<int>(belCenter.x + fx * KFTracker.statePost.at<float>(4) * 1.5, iWidth);
  int y2 = min<int>(belCenter.y + fy * KFTracker.statePost.at<float>(4) * 0, iHeight);
  flowROI.width = x2 - flowROI.x;
  flowROI.height = y2 - flowROI.y;

  cvtColor(rawFrameResized, rawFramGray, CV_BGR2GRAY);

  // TODO: Optimization here
  Mat _i1 = prevRawFrameGray;//(flowROI);
  Mat _i2 = rawFramGray;//(flowROI);

  int flags = 0;//OPTFLOW_USE_INITIAL_FLOW;
  if (firstCancel)
  {
    flags = 0;
    firstCancel = false;
  }
  calcOpticalFlowFarneback(_i1, _i2, flow, 0.5, 3, 10, 3, 9, 1.9, flags);  //OPTFLOW_USE_INITIAL_FLOW);
  std::vector<Mat> flowChannels;
  split(flow, flowChannels);

  // This mask is very important because zero-valued elements
  // Should not be taken into account
  Mat maskX;
  Mat maskY;
  Mat maskFull;

  maskX = abs(flowChannels[0]) > 0.01;
  maskY = abs(flowChannels[1]) > 0.01;
  bitwise_and(maskX, maskY, maskFull);

  // Gesture regions update
  gestureRegion[REG_TOPLEFT].x = flowROI.x;
  gestureRegion[REG_TOPLEFT].y = flowROI.y + (0.5 * flowROI.height);
  gestureRegion[REG_TOPLEFT].width = 0.5 * (flowROI.width - (fx * beleif.width));
  gestureRegion[REG_TOPLEFT].height = flowROI.height;

  // changed by Jake
  //gestureRegion[REG_TOPRIGHT].x = gestureRegion[REG_TOPLEFT].x + gestureRegion[REG_TOPLEFT].width + (fx * beleif.width);
  int _d = belCenter.x - (gestureRegion[REG_TOPLEFT].x + gestureRegion[REG_TOPLEFT].width);
  gestureRegion[REG_TOPRIGHT].x = belCenter.x + _d;
  gestureRegion[REG_TOPRIGHT].y = gestureRegion[REG_TOPLEFT].y;
  gestureRegion[REG_TOPRIGHT].width = gestureRegion[REG_TOPLEFT].width;
  gestureRegion[REG_TOPRIGHT].height = gestureRegion[REG_TOPLEFT].height;

  // Safety check to downsampled image size
  safeRectToImage(gestureRegion[REG_TOPLEFT], fx, fy);
  safeRectToImage(gestureRegion[REG_TOPRIGHT], fx, fy);

  // Cancel Ego Motion & Face bias As much As possible
  int _row = 0;
  int _col = 0;
  std::vector<cv::Point2f> prev;
  std::vector<cv::Point2f> curr;
  std::vector<cv::Point2f> prev_stab;
  Mat homography = Mat::eye(3, 3, CV_32F);

  magnitude(flowChannels[0], flowChannels[1], flowMag);

//    ROS_INFO("Sum Mag Before: %6.f", sum(flowMag)[0]);
  if (stablization == STABLIZE_MEDIAN)
  {

//    // The possible hand regions are not sampled for flow compenstation
//        // Initially we use inverted mask here
    Mat maskRegions = Mat::ones(maskX.rows, maskX.cols, CV_8UC1);
    Mat maskAugmentedX = Mat::ones(maskX.rows, maskX.cols, CV_8UC1);;
    Mat maskAugmentedY = Mat::ones(maskY.rows, maskY.cols, CV_8UC1);;

    rectangle(maskRegions, gestureRegion[REG_TOPLEFT].tl(), gestureRegion[REG_TOPLEFT].br(), 0, CV_FILLED);
    rectangle(maskRegions, gestureRegion[REG_TOPRIGHT].tl(), gestureRegion[REG_TOPRIGHT].br(), 0, CV_FILLED);

    double minX, minY, maxX, maxY;
    float medX = 0.0;
    float medY = 0.0;

    bitwise_and(maskX, maskRegions, maskAugmentedX);
    bitwise_and(maskY, maskRegions, maskAugmentedY);

    minMaxLoc(flowChannels[0], &minX, &maxX, 0, 0, maskAugmentedX);
    minMaxLoc(flowChannels[1], &minY, &maxY, 0, 0, maskAugmentedY);

    if ((maxX > minX) && (maxY > minY))
    {
      try
      {
        medX = calcMedian(flowChannels[0], 25, minX, maxX, maskAugmentedX);
      }
      catch (cv::Exception &e)
      {
        ROS_WARN("Calculate median failed for X flow with %s", e.what());
      }

      try
      {
        medY = calcMedian(flowChannels[1], 25, minY, maxY, maskAugmentedY);
      }
      catch (cv::Exception &e)
      {
        ROS_WARN("Calculate median failed for Yflow with %s", e.what());
      }



      ////    ROS_INFO("Number of channls: %d", flowChannels.size());
      ////        ROS_INFO("Ego");
      ////        ROS_INFO("X: <%6.4lf..%6.4lf> Y: <%6.4lf..%6.4lf>", minX, maxX, minY, maxY);
      ////        ROS_INFO("Median X: %6.4f Y: %6.4f", medX, medY);

      //        // Canceling Flow for all valid regions
      add(flowChannels[0], Scalar::all(-medX), flowChannels[0], maskX);
      add(flowChannels[1], Scalar::all(-medY), flowChannels[1], maskY);
    }
//        // Face Bias
    if (trackingState == STATE_TRACK)
    {
      Rect fROI = faces.at(0);
      fROI.x *= fx;
      fROI.y *= fy;
      fROI.width *= fx;
      fROI.height *= fy;
      safeRectToImage(fROI, fx, fy);

      Mat faceFlowWindowX = flowChannels[0](fROI);
      Mat faceFlowWindowY = flowChannels[1](fROI);
      Mat faceMaskX = maskX(fROI);
      Mat faceMaskY = maskY(fROI);

      minMaxLoc(faceFlowWindowX, &minX, &maxX, 0, 0, faceMaskX);
      minMaxLoc(faceFlowWindowY, &minY, &maxY, 0, 0, faceMaskY);
      if ((maxX > minX) && (maxY > minY))
      {
        try
        {
          medX = calcMedian(faceFlowWindowX, 25, minX, maxX, faceMaskX);
          medY = calcMedian(faceFlowWindowY, 25, minY, maxY, faceMaskY);

          // Now we want to cancel motion only in the interesting regions
          maskRegions = 1 - maskRegions;
          bitwise_and(maskX, maskRegions, maskAugmentedX);
          bitwise_and(maskY, maskRegions, maskAugmentedY);

          add(flowChannels[0], Scalar::all(-medX), flowChannels[0], maskAugmentedX);
          add(flowChannels[1], Scalar::all(-medY), flowChannels[1], maskAugmentedY);
        }
        catch (cv::Exception &e)
        {
          ROS_WARN("Face flow cancellation failed with %s", e.what());
        }
      }
    }
  }
  else if (stablization == STABLIZE_HOMOGRAPHY)
  {
//        Mat maskRegions = Mat::ones(maskX.rows, maskX.cols, CV_8UC1);
//        Mat maskAugmentedX = Mat::ones(maskX.rows, maskX.cols, CV_8UC1);;
//        Mat maskAugmentedY = Mat::ones(maskY.rows, maskY.cols, CV_8UC1);;

//        rectangle(maskRegions, gestureRegion[REG_TOPLEFT].tl(), gestureRegion[REG_TOPLEFT].br(), 0, CV_FILLED);
//        rectangle(maskRegions, gestureRegion[REG_TOPRIGHT].tl(), gestureRegion[REG_TOPRIGHT].br(), 0, CV_FILLED);

//        bitwise_and(maskX, maskRegions, maskAugmentedX);
//        bitwise_and(maskY, maskRegions, maskAugmentedY);

//        Mat maskFullAugmented;
//        bitwise_and(maskAugmentedX, maskAugmentedY, maskFullAugmented);

    for (_row = 0; _row < flow.rows; _row += 1)
    {
      for (_col = 0; _col < flow.cols; _col += 1)
      {
        if (maskFull.at<uchar>(_row, _col) == 0) continue;
        const Point2f& prev_point = (1.0 / fx) * (Point2f(_col, _row) - flow.at<Point2f>(_row, _col));
        const Point2f& curr_point = (1.0 / fx) * Point2f(_col, _row);
        prev.push_back(prev_point);
        curr.push_back(curr_point);
      }
    }

    try
    {
      homography = findHomography(prev, curr, CV_RANSAC, 3);


      //        cv::warpPerspective(_i1, _i1, homography, Size(_i1.cols, _i1.rows), cv::WARP_INVERSE_MAP);
      //        cv::warpPerspective(_i2, _i2, homography, Size(_i2.cols, _i2.rows), cv::WARP_INVERSE_MAP);
      //        calcOpticalFlowFarneback( _i1, _i2 , flow, 0.5, 3, 5, 3, 9, 1.9, 0);//OPTFLOW_USE_INITIAL_FLOW);
      //        split(flow, flowChannels);

      perspectiveTransform(curr, prev_stab, homography.inv());


      for (unsigned int i = 0; i < curr.size(); i++)
      {
        //            std::cout << "Current: " << curr[i] << std::endl;
        //            std::cout << "Prev Stab: " << prev_stab[i] << std::endl;
        const Point2f& flow_extra = curr[i] - prev_stab[i];
        //            std::cout << "Extra Flow: " << flow_extra << std::endl;
        const unsigned int _row = curr[i].y * fy;
        const unsigned int _col = curr[i].x * fx;
        //            std::cout << "Current Flow: " << flow.at<Point2f>(_row, _col) << std::endl;
        //            ROS_INFO("Decreasing the flow at %3d %3d from %4.2f %4.2f by %4.2f %4.2f",
        //                     _row, _col,
        //                     flowChannels[1].at<float>(_row, _col),
        //                     flowChannels[0].at<float>(_row, _col),
        //                     flow_extra.y, flow_extra.x);
        //            if (
        //                ((flowChannels[0].at<float>(_row, _col) * flow_extra.x) > 0) &&
        //                ((flowChannels[1].at<float>(_row, _col) * flow_extra.y) > 0)
        //                ) cc++;
        flowChannels[0].at<float>(_row, _col) -= (flow_extra.x * fx);
        flowChannels[1].at<float>(_row, _col) -= (flow_extra.y * fy);
      }
      //        ROS_INFO("T: %5d / %5d", cc, curr.size());
      //        for (_row = 0; _row < flow.rows; _row++) {
      //            for (_col = 0; _col < flow.cols; _col++) {
      //                flowChannels[0].at<float>(_row, _col) = prev_stab
      //            }
      //        }
    }
    catch (cv::Exception &e)
    {
      ROS_WARN("Findhomography failed with %s", e.what());
    }
  }
  magnitude(flowChannels[0], flowChannels[1], flowMag);


  if (skinEnabled)
  {
    Mat skin_small;
    resize(skin, skin_small, Size(0, 0), fx, fy);
    normalize(skin_small, skin_small, 1.0, 0.0, NORM_MINMAX);
    Mat skin_mask = skin_small > 0.7;
    multiply(flowMag, skin_mask, flowMag, 1.0, CV_32FC1);
  }
  else
  {
    ;//threshold(flowMag, flowMag, (double) minFlow, 0.0, THRESH_TOZERO);
  }
//    ROS_INFO("Sum Mag After: %6.f", sum(flowMag)[0]);



  // Big Question: Why does the below line make the signal so weak?
//  normalize(flowMag, flowMag, 0.0, 1.0, NORM_MINMAX);

  Rect r;

  //std::swap(prevRawFrameGray, rawFramGray);
  prevRawFrameGray = rawFramGray.clone();

  for (int i = 0; i < 2; i++)
  {

    Rect reg = gestureRegion[i];
    reg.x = gestureRegion[i].x;// - flowROI.x;
    reg.y = gestureRegion[i].y;// - flowROI.y;

    float sFlow = 0.0;
    if (skinEnabled)
    {
      sFlow = ((gestureRegion[i].width > 0) && (gestureRegion[i].height > 0)) ?
              mean(flowMag(reg), maskFull(reg))[0] : 0.0;
    }
    else
    {
      // Threshold and mean over thresholded values
      sFlow = ((gestureRegion[i].width > 0) && (gestureRegion[i].height > 0)) ?
              mean(flowMag(reg), (flowMag(reg) > minFlow))[0] : 0.0;
    }


    //flowScoreInRegion[i] = (0.5 * flowScoreInRegion[i]) + (0.5 * sFlow);
    // No filtering should be done here
    flowScoreInRegion[i] = sFlow;
  }

  // Visulization
  if ((debugLevel & 0x10) == 0x10)
  {
    std::vector<Mat> channels;
    split(rawFrameResized, channels);

    // HSV Color Space
    // Red to blue specterum is between 0->120 degrees (Red:0, Blue:120)
    flowMag.convertTo(channels[0], CV_8UC1, 10);
    channels[0] = Scalar::all(255) - channels[0];
    //flowMag = 120 - flowMag;
    channels[1] = Scalar::all(255.0);
    channels[2] = _i2;//Scalar::all(255.0);
    merge(channels, opticalFrame);
    cvtColor(opticalFrame, opticalFrame, CV_HSV2BGR);
    if (false) //(stablization == STABLIZE_HOMOGRAPHY) {
    {
      for (unsigned int i = 0; i < prev.size(); i++)
      {
        line(debugFrame, prev[i], curr[i], CV_RGB(0, 255, 0));
        //            line(debugFrame, prev_stab[i], curr[i], CV_RGB(255,0,0));
        //            circle(opticalFrame, curr[i], 1, CV_RGB(255,255,255));
      }
    }
  }

  if ((debugLevel & 0x02) == 0x02)
  {
    if (stablization == STABLIZE_HOMOGRAPHY)
      cv::warpPerspective(debugFrame, debugFrame, homography, Size(debugFrame.cols, debugFrame.rows), cv::WARP_INVERSE_MAP);
    //cv::warpAffine(debugFrame, debugFrame, homography(Rect(0,0,3,2)), Size(debugFrame.cols, debugFrame.rows), cv::WARP_INVERSE_MAP);

    //rectangle(debugFrame, Rect(flowROI.x / fx, flowROI.y / fy, flowROI.width / fx, flowROI.height / fy), CV_RGB(255, 0, 0));
    for (int i = 0; i < 2; i++)
    {
      // Only for visualization
      gestureRegion[i].x /= fx;
      gestureRegion[i].y /= fy;
      gestureRegion[i].width /= fx;
      gestureRegion[i].height /= fy;

      std::stringstream txtstr;
      rectangle(debugFrame, gestureRegion[i], CV_RGB(0, 0, 0));
      Point2d center;
      center.x = gestureRegion[i].x + gestureRegion[i].width / 2;
      center.y = gestureRegion[i].y + gestureRegion[i].height / 2;
      txtstr << fixed << setprecision(3) << flowScoreInRegion[i];
//      putText(debugFrame, txtstr.str(), center, FONT_HERSHEY_PLAIN, 1, CV_RGB(0,0,255));
    }
  }



}

void CHumanTracker::draw()
{
  return ;
}

void CHumanTracker::visionCallback(const sensor_msgs::ImageConstPtr& frame)
{
  tStart = ros::Time::now();
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    frame_id = frame->header.frame_id;
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
    reset();
  }

  //TODO: Check clone
  this->rawFrame = cv_ptr->image;

  tFaceStart = ros::Time::now();
  detectAndTrackFace();

  tSkinStart = ros::Time::now();
  trackSkin();

  tFlowStart = ros::Time::now();
  calcOpticalFlow();

  publish();
  tEnd = ros::Time::now();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomy_human");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  string p_xmlFile, p_xmlFileProfile;

  if (false == ros::param::get("~cascade_file", p_xmlFile))
  {
    ROS_FATAL("No cascade file provided, use `cascade_file` param to set it.");
    ros::shutdown();
    exit(1);
  }
  else
  {
    ROS_INFO("Cascade file: %s", p_xmlFile.c_str());
  }

  bool p_profileFaceEnabled;
  ros::param::param("~profile_hack_enabled", p_profileFaceEnabled, false);
  ROS_INFO("Profile Face Hack is %s", p_profileFaceEnabled ? "Enabled" : "Disabled");

  if ((false == ros::param::get("~cascade_profile_file", p_xmlFileProfile)) && (p_profileFaceEnabled))
  {
    ROS_FATAL("No profile cascade file provided, use `cascade_profile_file` param to set it.");
    ros::shutdown();
    exit(2);
  }
  else
  {
    ROS_INFO("Profile Cascade file: %s", p_xmlFileProfile.c_str());
  }



  bool p_skinEnabled;
  ros::param::param("~skin_enabled", p_skinEnabled, false);
  ROS_INFO("Skin Segmentation is %s", p_skinEnabled ? "Enabled" : "Disabled");

  bool p_gestureEnabled;
  ros::param::param("~gesture_enabled", p_gestureEnabled, false);
  ROS_INFO("Gesture Recognition is %s", p_gestureEnabled ? "Enabled" : "Disabled");

  int p_stablization;
  ros::param::param("~flowstablize_mode", p_stablization, 0);
  ROS_INFO("Flow Stablization is %d", p_stablization);

  int  p_debugMode;
  ros::param::param("~debug_mode", p_debugMode, 0x02);
  ROS_INFO("Debug mode is %x", p_debugMode);

  int p_initialScoreMin, p_initialDetectFrames, p_initialRejectFrames, p_minFlow;
  ros::param::param("~initial_min_score", p_initialScoreMin, 5);
  ros::param::param("~initial_detect_frames", p_initialDetectFrames, 6);
  ros::param::param("~initial_reject_frames", p_initialRejectFrames, 6);
  ros::param::param("~min_flow", p_minFlow, 10);


  int p_minFaceSizeW, p_minFaceSizeH, p_maxFaceSizeW, p_maxFaceSizeH;
  ros::param::param("~min_face_width", p_minFaceSizeW, 12);
  ros::param::param("~min_face_height", p_minFaceSizeH, 18);
  ros::param::param("~max_face_width", p_maxFaceSizeW, 60);
  ros::param::param("~max_face_height", p_maxFaceSizeH, 80);

  double p_mCov, p_pCov;
  ros::param::param("~meas_cov", p_mCov, 1.0);
  ros::param::param("~proc_cov", p_pCov, 0.05);




  /**
   * The queue size seems to be very important in this project
   * If we can not complete the calculation in the 1/fps time slot, we either
   * drop the packet (low queue size) or process all frames which will lead to
   * additive delays.
   */


  ros::Publisher facePub = n.advertise<autonomy_human::human>("human", 5);
  ros::Publisher allDetectionsPub = n.advertise<autonomy_human::raw_detections>("raw_detections", 5);
  image_transport::Publisher debugPub = it.advertise("output_rgb_debug", 1);
  image_transport::Publisher skinPub = it.advertise("output_rgb_skin", 1);
  image_transport::Publisher opticalPub = it.advertise("output_rgb_optical", 1);

  CHumanTracker humanTracker(p_xmlFile, p_xmlFileProfile,
                             p_pCov, p_mCov,
                             p_minFaceSizeW, p_minFaceSizeH, p_maxFaceSizeW, p_maxFaceSizeH,
                             p_initialScoreMin, p_initialDetectFrames, p_initialRejectFrames, p_minFlow,
                             p_profileFaceEnabled, p_skinEnabled, p_gestureEnabled,
                             p_debugMode, p_stablization,
                             facePub, allDetectionsPub, debugPub, skinPub, opticalPub);

  image_transport::Subscriber visionSub = it.subscribe("input_rgb_image", 1, &CHumanTracker::visionCallback, &humanTracker);

  ROS_INFO("Starting Autonomy Human ...");

  ros::Rate loopRate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

