#include <sys/stat.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "autonomy_human/human.h"
#include "autonomy_human/raw_detections.h"

class CHumanTracker
{
public:
  enum tracking_state_t
  {
    STATE_LOST = 0,
    STATE_DETECT = 1,
    STATE_TRACK = 2,
    STATE_REJECT = 3,
    STATE_NUM = 4
  };

  enum gesture_regions_ids_t
  {
    REG_TOPLEFT = 0,
    REG_TOPRIGHT = 1,
    REG_NUM = 2
  };

  enum stabilization_methods_t
  {
    STABLIZE_OFF = 0,
    STABLIZE_MEDIAN,
    STABLIZE_HOMOGRAPHY,
    STABLIZAE_NUM
  };

private:
  bool is_enabled_;
  int32_t image_width_;
  int32_t image_height_;
  //ros::NodeHandle node;
  cv::Mat frame_raw_;
  cv::Rect search_roi_;
  cv::Rect flow_roi_;
  cv::Mat frame_raw_gray_;
  cv::Mat frame_raw_gray_prev_;

  // Face Tracker
  cv::KalmanFilter kf_tracker_;
  cv::KalmanFilter ml_search_; // Maximum Likelihood Search for multiple faces
  cv::Mat state_; // x,y,xdot,ydot,w,h
  cv::Mat measurement_;
  float p_cov_scalar_;
  float m_cov_scalar_;

  // State Machine
  int32_t state_counter_;
  int32_t initial_score_min_;
  int32_t min_detect_frames_;
  int32_t min_reject_frames_;
  float max_reject_cov_;
  cv::Size min_face_size_;
  cv::Size max_face_size_;

  // Histograms (Skin)
  cv::MatND face_hist_;
  int32_t hbins_;
  int32_t sbins_;
  cv::Mat skin_;
  cv::Mat skin_prior_;
  cv::Mat flow_;
  cv::Mat flow_mag_;

  // Optical Flow
  int32_t min_flow_;

  //Time measurements
  ros::Time t_start_;
  ros::Time t_face_start_;
  ros::Time t_skin_start_;
  ros::Time t_flow_start_;
  ros::Time t_end_;

  std::string str_states_[4];

  /*
   * Bit 0: Original Image
   * Bit 1: Face Image
   * Bit 2: Skin Image
     * Bit 3: Histogram
   * Bit 4: Optical Flow
   */
  uint8_t debug_level_;
  bool profile_hack_enabled_;
  bool skin_enabled_;
  bool gesture_enabled_;

  // C API is needed for score
  // But let's add smart pointers
  cv::Ptr<CvHaarClassifierCascade> cascade_;
  cv::Ptr<CvHaarClassifierCascade> cascade_profile_;

  void CopyKalman(const cv::KalmanFilter& src, cv::KalmanFilter& dest);
  void ResetKalmanFilter();
  void InitMats();
  void ResetMats();
  void GenerateRegionHistogram(cv::Mat& region, cv::MatND &hist, bool vis = true);
  void HistFilter(cv::Mat& region, cv::Mat& post, cv::Mat& prior, cv::MatND& hist,  bool vis = true);
  void DetectAndTrackFace();
  void TrackSkin();
  void CalcOpticalFlow();
  void Draw();
  void SafeRect(cv::Rect &r, cv::Rect &boundry);
  void SafeRectToImage(cv::Rect &r, const double fx = 1.0, const double fy = 1.0);

  // Temp
  CvAvgComp ml_face_;
  std::string frame_id_;

  // We need C API to get score
  // Let's use OpenCV Smart Pointers to old data structures
  cv::Ptr<CvMemStorage> storage_;
  cv::Ptr<CvMemStorage> storage_profile_;

  ros::Publisher& face_pub_;
  ros::Publisher& all_detections_pub_;
  image_transport::Publisher& debug_pub_;
  image_transport::Publisher& skin_pub_;
  image_transport::Publisher& optical_pub_;

public:
  CHumanTracker(const bool enabled, std::string &cascadeFile, std::string &cascadeFileProfile,
                float _pCov, float _mCov,
                int _minFaceSizeW, int _minFaceSizeH, int _maxFaceSizeW, int _maxFaceSizeH,
                int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, int _minFlow,
                bool _profileHackEnabled, bool _skinEnabled, bool _gestureEnabled,
                unsigned short int _debugLevel, unsigned int _stablization,
                ros::Publisher& _facePub, ros::Publisher& _allDetectionsPub,
                image_transport::Publisher& _debugPub, image_transport::Publisher& _skinPub, image_transport::Publisher& _opticalPub);

  void Publish();
  ~CHumanTracker();

  void EnableCallback(const std_msgs::BoolConstPtr& enable);
  void VisionCallback(const sensor_msgs::ImageConstPtr& frame);
  void Reset();
  float CalcMedian(cv::Mat &m, int nbins, float minVal, float maxVal, cv::InputArray mask = cv::noArray());

  bool is_inited_;
  cv::Mat frame_debug_;
  cv::Mat frame_skin_;
  cv::Mat frame_optical_;

  std::vector<cv::Rect> faces_;
  int32_t face_score_;
  cv::Rect beleif_;
  tracking_state_t tracking_state_;
  bool is_face_in_current_frame_;

  bool should_publish_;
  uint32_t stab_;

  /*
   * Drone Point of View, Face in center
   * 0 : Top left
   * 1 : Top right
   */
  cv::Rect gesture_region_[2];
  float flow_score_in_region_[2];

};

//
// KFTracker = new KalmanFilter(6, 4, 0);
// MLSearch = new KalmanFilter(6, 4, 0);
CHumanTracker::CHumanTracker(const bool enabled, std::string &cascadeFile, std::string &cascadeFileProfile,
                             float _pCov, float _mCov,
                             int _minFaceSizeW, int _minFaceSizeH, int _maxFaceSizeW, int _maxFaceSizeH,
                             int _initialScoreMin, int _initialDetectFrames, int _initialRejectFrames, int _minFlow,
                             bool _profileHackEnabled, bool _skinEnabled, bool _gestureEnabled,
                             unsigned short int _debugLevel, unsigned int _stablization,
                             ros::Publisher& _facePub, ros::Publisher& _allDetectionsPub,
                             image_transport::Publisher& _debugPub, image_transport::Publisher& _skinPub, image_transport::Publisher& _opticalPub)
  : is_enabled_(enabled)
  , kf_tracker_(6, 4, 0)
  , ml_search_(6, 4, 0)
  , p_cov_scalar_(_pCov)
  , m_cov_scalar_(_mCov)
  , state_counter_(0)
  , initial_score_min_(_initialScoreMin)
  , min_detect_frames_(_initialDetectFrames)
  , min_reject_frames_(_initialRejectFrames)
  , max_reject_cov_(6.0)
  , min_face_size_(_minFaceSizeW, _minFaceSizeH)
  , max_face_size_(_maxFaceSizeW, _maxFaceSizeH)
  , hbins_(15)
  , sbins_(16)
  , min_flow_(_minFlow)
  , debug_level_(_debugLevel)
  , profile_hack_enabled_(_profileHackEnabled)
  , skin_enabled_(_skinEnabled)
  , gesture_enabled_(_gestureEnabled)
  , storage_(cvCreateMemStorage(0))
  , storage_profile_(cvCreateMemStorage(0))
  , face_pub_(_facePub)
  , all_detections_pub_(_allDetectionsPub)
  , debug_pub_(_debugPub)
  , skin_pub_(_skinPub)
  , optical_pub_(_opticalPub)
  , is_inited_(false)
  , tracking_state_(STATE_LOST)
  , should_publish_(false)
  , stab_(_stablization)
{

  str_states_[0] = "NOFACE";
  str_states_[1] = "DETECT";
  str_states_[2] = "TRACKG";
  str_states_[3] = "REJECT";

  struct stat stat_buffer;
  const bool c_exists = (stat(cascadeFile.c_str(), &stat_buffer) == 0);

  if (c_exists)
  {
    cascade_ = (CvHaarClassifierCascade*) cvLoad(cascadeFile.c_str(), 0, 0, 0);
  }

  if (!c_exists || cascade_.empty())
  {
    ROS_FATAL("Problem loading cascade file %s", cascadeFile.c_str());
  }

  if (profile_hack_enabled_)
  {
    const bool c_exists = (stat(cascadeFileProfile.c_str(), &stat_buffer) == 0);
    if (c_exists)
      cascade_profile_ = (CvHaarClassifierCascade*) cvLoad(cascadeFileProfile.c_str(), 0, 0, 0);

    if (!c_exists || cascade_.empty())
    {
      ROS_FATAL("Problem loading profile cascade file %s", cascadeFileProfile.c_str());
    }
  }

  is_face_in_current_frame_ = false;

}

void CHumanTracker::Publish()
{
  autonomy_human::human msg;
  autonomy_human::raw_detections faces_msg;

  cv_bridge::CvImage cvi;
  sensor_msgs::Image im;
  cvi.header.frame_id = "image";

  if  (
        (tracking_state_ == CHumanTracker::STATE_TRACK) ||
        (tracking_state_ == CHumanTracker::STATE_REJECT)
      )
  {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.numFaces = faces_.size();
    msg.faceScore = face_score_;

    if (
        (tracking_state_ == CHumanTracker::STATE_TRACK) ||
        (tracking_state_ == CHumanTracker::STATE_REJECT)
        )
    {
      msg.status = autonomy_human::human::STATUS_TRACKING;
      msg.faceROI.x_offset = beleif_.x;
      msg.faceROI.y_offset = beleif_.y;
      msg.faceROI.width = beleif_.width;
      msg.faceROI.height = beleif_.height;
      if (gesture_enabled_)
      {
        for (int i = 0; i < 2; i++)
        {
          msg.flowScore[i] = flow_score_in_region_[i];
          if( i)
          {
            msg.leftHROI.x_offset = gesture_region_[i].x;
            msg.leftHROI.y_offset = gesture_region_[i].y;
            msg.leftHROI.width = gesture_region_[i].width;
            msg.leftHROI.height = gesture_region_[i].height;
          }
          else
          {
            msg.rightHROI.x_offset = gesture_region_[i].x;
            msg.rightHROI.y_offset = gesture_region_[i].y;
            msg.rightHROI.width = gesture_region_[i].width;
            msg.rightHROI.height = gesture_region_[i].height;
          }
        }
      }
    }
    else
    {
      msg.status = (is_inited_) ? autonomy_human::human::STATUS_LOST : autonomy_human::human::STATUS_UNKNOWN;
      msg.faceROI.x_offset = 0;
      msg.faceROI.y_offset = 0;
      msg.faceROI.width = 0;
      msg.faceROI.height = 0;
      for (int i = 0; i < 2; i++)
        msg.flowScore[i] = 0;
    }

    face_pub_.publish(msg);
  }


  if (all_detections_pub_.getNumSubscribers() > 0)
  {
    faces_msg.header.stamp = ros::Time::now();
    faces_msg.header.frame_id = frame_id_;

    for (size_t i = 0; i < faces_.size(); i++)
    {
      sensor_msgs::RegionOfInterest tmp_human;
      tmp_human.x_offset = faces_[i].x;
      tmp_human.y_offset = faces_[i].y;
      tmp_human.width = faces_[i].width;
      tmp_human.height = faces_[i].height;
      faces_msg.detections.push_back(tmp_human);
    }

    all_detections_pub_.publish(faces_msg);
  }

  if (
    (debug_pub_.getNumSubscribers() > 0) &&
    (should_publish_) &&
    ((debug_level_ & 0x02) == 0x02) &&
    (is_inited_)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "bgr8";
    cvi.image = frame_debug_;
    cvi.toImageMsg(im);
    debug_pub_.publish(im);
  }

  if (
    (skin_pub_.getNumSubscribers() > 0) &&
    (should_publish_) &&
    ((debug_level_ & 0x04) == 0x04) &&
    (is_inited_) &&
    (skin_enabled_)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "mono8";
    cvi.image = frame_skin_;
    cvi.toImageMsg(im);
    skin_pub_.publish(im);
  }

  if (
    (optical_pub_.getNumSubscribers() > 0) &&
    (should_publish_) &&
    ((debug_level_ & 0x10) == 0x10) &&
    (is_inited_) &&
    (gesture_enabled_)
  )
  {
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "bgr8";
    cvi.image = frame_optical_;
    cvi.toImageMsg(im);
    optical_pub_.publish(im);
  }
}

CHumanTracker::~CHumanTracker()
{
  ;
}

void CHumanTracker::CopyKalman(const cv::KalmanFilter& src, cv::KalmanFilter& dest)
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

void CHumanTracker::Reset()
{
  tracking_state_ = STATE_LOST;
  search_roi_ = cv::Rect(cv::Point(0, 0), cv::Point(image_width_, image_height_));

  // x,y,xdot,ydot,w,h
  state_ = cv::Mat::zeros(6, 1, CV_32F);

  // x,y,w,h
  measurement_ = cv::Mat::zeros(4, 1, CV_32F);

  is_face_in_current_frame_ = false;

  for (int i = 0; i < 2; i++)
    flow_score_in_region_[i] = 0.0;

  InitMats();
  ResetMats();
  ResetKalmanFilter();
}

void CHumanTracker::InitMats()
{
  skin_ = cv::Mat::zeros(image_height_, image_width_, CV_32F);
  skin_prior_ = cv::Mat::zeros(image_height_, image_width_, CV_32F);
  face_hist_ = cv::Mat::zeros(hbins_, sbins_, CV_32F);
  frame_skin_ = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);
  frame_optical_ = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  flow_ = cv::Mat::zeros(image_height_, image_width_, CV_32FC2);
  flow_mag_ = cv::Mat::zeros(image_height_, image_width_, CV_32F);
}

void CHumanTracker::ResetMats()
{
  skin_prior_ = cv::Scalar::all(1.0 / (image_width_ * image_height_));
  skin_ = cv::Scalar::all(1.0 / (skin_.rows * skin_.cols));
  face_hist_ = cv::Scalar::all(1.0 / (face_hist_.rows * face_hist_.cols));
}
void CHumanTracker::ResetKalmanFilter()
{
  measurement_.setTo(cv::Scalar(0));

  // The system model is very naive. The small effect of xdot and ydot
  // on x and y are intentional (it is 0 now)
  kf_tracker_.transitionMatrix = (cv::Mat_<float>(6, 6)
                                <<
                                1, 0, 0, 0, 0, 0,
                                0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0,
                                0, 0, 0, 1, 0, 0,
                                0, 0, 0, 0, 1, 0,
                                0, 0, 0, 0, 0, 1
                               );

  kf_tracker_.measurementMatrix = (cv::Mat_<float>(4, 6)
                                 <<
                                 1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1
                                );

  kf_tracker_.statePre = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);
  kf_tracker_.statePost = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);

  setIdentity(kf_tracker_.errorCovPre, cv::Scalar_<float>::all(1e2));
  setIdentity(kf_tracker_.errorCovPost, cv::Scalar_<float>::all(1e2));
  setIdentity(kf_tracker_.processNoiseCov, cv::Scalar_<float>::all(p_cov_scalar_));
  setIdentity(kf_tracker_.measurementNoiseCov, cv::Scalar_<float>::all(m_cov_scalar_));

  CopyKalman(kf_tracker_, ml_search_);


}

float CHumanTracker::CalcMedian(cv::Mat &m, int nbins, float minVal, float maxVal, cv::InputArray mask)
{
  cv::MatND hist;
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

void CHumanTracker::SafeRect(cv::Rect& r, cv::Rect& boundry)
{
  r.x = std::max<int>(r.x, boundry.x);
  r.y = std::max<int>(r.y, boundry.y);

  cv::Point2d p2;
  p2.x = std::min<int>(r.br().x, boundry.br().x);
  p2.y = std::min<int>(r.br().y, boundry.br().y);

  r.width = p2.x - r.x;
  r.height = p2.y - r.y;
}

void CHumanTracker::SafeRectToImage(cv::Rect& r, const double fx, const double fy)
{
  cv::Rect safety(0, 0, image_width_ * fx, image_height_ * fy);
  SafeRect(r, safety);
}

void CHumanTracker::GenerateRegionHistogram(cv::Mat& region, cv::MatND &hist, bool vis)
{
  cv::MatND prior = hist.clone();

  cv::Mat hsv;
  cvtColor(region, hsv, CV_BGR2HSV);

  cv::Mat mask = cv::Mat::zeros(region.rows, region.cols, CV_8UC1);
  for (int r = 0; r < region.rows; r++)
  {
    for (int c = 0; c < region.cols; c++)
    {
      unsigned int v = hsv.at<cv::Vec3b>(r, c)[2];
      // TODO: Make me parameters
      if ((v > 10) && (v < 240))
      {
        mask.at<uchar>(r, c) = 1;
      }
    }
  }

//  namedWindow( "Face Mask", 1 );
//  imshow( "Face Mask", mask * 255 );

  int histSize[] = {hbins_, sbins_};
  float hranges[] = {0, 180};
  float sranges[] = {0, 256};
  const float* ranges[] = { hranges, sranges};
  int channels[] = {0, 1};
  calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);

  for (int h = 0; h < hbins_; h++)
  {
    for (int s = 0; s < sbins_; s++)
    {
      hist.at<float>(h, s) = (0.99 * prior.at<float>(h, s)) + (0.01 * hist.at<float>(h, s));
    }
  }

  // We should make it a probability dist.
  normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);

  if (vis)
  {
    // For vis
    cv::Mat visHist;
    cv::normalize(hist, visHist, 255.0, 0.00, cv::NORM_MINMAX);

    int scale = 10;
    cv::Mat histImg = cv::Mat::zeros(sbins_ * scale, hbins_ * scale, CV_8UC3);
    for (int h = 0; h < hbins_; h++)
    {
      for (int s = 0; s < sbins_; s++)
      {
        float binVal = visHist.at<float>(h, s);
        int intensity = cvRound(binVal);///maxVal);
        cv::rectangle(histImg, cv::Point(h * scale, s * scale),
                  cv::Point((h + 1)*scale - 1, (s + 1)*scale - 1),
                  cv::Scalar::all(intensity),
                  CV_FILLED);
      }
    }

    if ((debug_level_ & 0x08) == 0x08)
    {
      cv::namedWindow("H-S Histogram", 1);
      cv::imshow("H-S Histogram", histImg);
      cv::waitKey(1);
    }
  }
}

void CHumanTracker::HistFilter(cv::Mat& region, cv::Mat& post, cv::Mat& prior, cv::MatND& hist,  bool vis)
{
  int _w = region.cols;
  int _h = region.rows;
  cv::Mat image;
  cv::cvtColor(region, image, CV_BGR2HSV);
  for (int r = 0; r < _h; r++)
  {
    for (int c = 0; c < _w; c++)
    {
      int hue_bin = cvFloor(image.at<cv::Vec3b>(r, c)[0] / (180.0 / (float) hbins_));
      int sat_bin = cvFloor(image.at<cv::Vec3b>(r, c)[1] / (256.0 / (float) sbins_));

      float z = hist.at<float>(hue_bin, sat_bin);
      post.at<float>(r, c) = (0.7 * z) + (0.3 * prior.at<float>(r, c));
      //post.at<float>(r,c) = z * prior.at<float>(r,c);
    }
  }

  if (vis)
  {
    // For vis;
    cv::Mat visPost;
    cv::normalize(post, visPost, 1.0, 0.0, cv::NORM_MINMAX);
    cv::namedWindow("Skin", 1);
    cv::imshow("Skin", visPost);
  }
  //prior = post.clone();
}

void CHumanTracker::DetectAndTrackFace()
{
  static ros::Time probe;

  // Do ROI
  frame_debug_ = frame_raw_.clone();
  cv::Mat img =  this->frame_raw_(search_roi_);

  faces_.clear();
  std::ostringstream txtstr;
  const static cv::Scalar colors[] =  { CV_RGB(0, 0, 255),
                                    CV_RGB(0, 128, 255),
                                    CV_RGB(0, 255, 255),
                                    CV_RGB(0, 255, 0),
                                    CV_RGB(255, 128, 0),
                                    CV_RGB(255, 255, 0),
                                    CV_RGB(255, 0, 0),
                                    CV_RGB(255, 0, 255)
                                  } ;
  cv::Mat gray;
  cv::Mat frame(cvRound(img.rows), cvRound(img.cols), CV_8UC1);
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  cv::resize(gray, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
  //equalizeHist( frame, frame );

  // This if for internal usage
  const ros::Time _n = ros::Time::now();
  double dt = (_n - probe).toSec();
  probe = _n;


  CvMat _image = frame;

  if (!storage_.empty())
  {
    cvClearMemStorage(storage_);
  }
  CvSeq* _objects = cvHaarDetectObjects(&_image, cascade_, storage_,
                                        1.2, initial_score_min_, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_SCALE_IMAGE, min_face_size_, max_face_size_);

  std::vector<CvAvgComp> vecAvgComp;
  cv::Seq<CvAvgComp>(_objects).copyTo(vecAvgComp);

  // End of using C API

  is_face_in_current_frame_ = (vecAvgComp.size() > 0);

  // This is a hack
  bool isProfileFace = false;
  if ((profile_hack_enabled_) && (!is_face_in_current_frame_) && ((tracking_state_ == STATE_REJECT) || (tracking_state_ == STATE_REJECT)))
  {
    ROS_DEBUG("Using Profile Face hack ...");

    if (!storage_profile_.empty())
    {
      cvClearMemStorage(storage_profile_);
    }
    CvSeq* _objectsProfile = cvHaarDetectObjects(&_image, cascade_profile_, storage_profile_,
                             1.2, initial_score_min_, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_SCALE_IMAGE, min_face_size_, max_face_size_);
    vecAvgComp.clear();
    cv::Seq<CvAvgComp>(_objectsProfile).copyTo(vecAvgComp);
    is_face_in_current_frame_ = (vecAvgComp.size() > 0);
    if (is_face_in_current_frame_)
    {
      ROS_DEBUG("The hack seems to work!");
    }
    isProfileFace = true;
  }

  if (tracking_state_ == STATE_LOST)
  {
    if (is_face_in_current_frame_)
    {
      state_counter_++;
      tracking_state_ = STATE_DETECT;
    }
  }

  if (tracking_state_ == STATE_DETECT)
  {
    if (is_face_in_current_frame_)
    {
      state_counter_++;
    }
    else
    {
      state_counter_ = 0;
      tracking_state_ = STATE_LOST;
    }

    if (state_counter_ > min_detect_frames_)
    {
      state_counter_ = 0;
      tracking_state_ = STATE_TRACK;
    }

  }

  if (tracking_state_ == STATE_TRACK)
  {
    if (!is_face_in_current_frame_)
    {
      tracking_state_ = STATE_REJECT;
    }
  }

  if (tracking_state_ == STATE_REJECT)
  {
    float covNorm = sqrt(
                      pow(kf_tracker_.errorCovPost.at<float>(0, 0), 2) +
                      pow(kf_tracker_.errorCovPost.at<float>(1, 1), 2)
                    );

    if (!is_face_in_current_frame_)
    {
      state_counter_++;
    }
    else
    {
      state_counter_ = 0;
      tracking_state_ = STATE_TRACK;
    }

    if ((state_counter_ > min_reject_frames_) && (covNorm > max_reject_cov_))
    {
      tracking_state_ = STATE_LOST;
      state_counter_ = 0;
      ResetKalmanFilter();
      Reset();
    }
  }

  if ((tracking_state_ == STATE_TRACK) || (tracking_state_ == STATE_REJECT))
  {
    bool updateFaceHist = false;
    // This is important:

    kf_tracker_.transitionMatrix.at<float>(0, 2) = dt;
    kf_tracker_.transitionMatrix.at<float>(1, 3) = dt;

    cv::Mat pred = kf_tracker_.predict();

    if (is_face_in_current_frame_)
    {
      //std::cout << vecAvgComp.size() << " detections in image " << std::endl;
      float minCovNorm = 1e24;
      int i = 0;
      for (std::vector<CvAvgComp>::const_iterator rr = vecAvgComp.begin(); rr != vecAvgComp.end(); rr++, i++)
      {
        CopyKalman(kf_tracker_, ml_search_);
        CvRect r = rr->rect;
        r.x += search_roi_.x;
        r.y += search_roi_.y;
        double nr = rr->neighbors;
        cv::Point center;
        cv::Scalar color = colors[i % 8];

        float normFaceScore = 1.0 - (nr / 40.0);
        if (normFaceScore > 1.0) normFaceScore = 1.0;
        if (normFaceScore < 0.0) normFaceScore = 0.0;
        setIdentity(ml_search_.measurementNoiseCov, cv::Scalar_<float>::all(normFaceScore));

        center.x = cvRound(r.x + r.width * 0.5);
        center.y = cvRound(r.y + r.height * 0.5);

        measurement_.at<float>(0) = r.x;
        measurement_.at<float>(1) = r.y;
        measurement_.at<float>(2) = r.width;
        measurement_.at<float>(3) = r.height;

        ml_search_.correct(measurement_);

        float covNorm = sqrt(
                          pow(ml_search_.errorCovPost.at<float>(0, 0), 2) +
                          pow(ml_search_.errorCovPost.at<float>(1, 1), 2)
                        );

        if (covNorm < minCovNorm)
        {
          minCovNorm = covNorm;
          ml_face_ = *rr;
        }

//                if ((debugLevel & 0x02) == 0x02)
//                {
        rectangle(frame_debug_, center - cv::Point(r.width * 0.5, r.height * 0.5),
                  center + cv::Point(r.width * 0.5, r.height * 0.5), color);

        txtstr.str("");
        txtstr << "   Sc:" << rr->neighbors << " S:" << r.width << "x" << r.height;

        cv::putText(frame_debug_, txtstr.str(), center, cv::FONT_HERSHEY_PLAIN, 1, color);
//                }
      }

      cv::Rect r(ml_face_.rect);
      r.x += search_roi_.x;
      r.y += search_roi_.y;
      faces_.push_back(r);
      double nr = ml_face_.neighbors;
      face_score_ = nr;
      if (isProfileFace) face_score_ = 0.0;
      float normFaceScore = 1.0 - (nr / 40.0);
      if (normFaceScore > 1.0) normFaceScore = 1.0;
      if (normFaceScore < 0.0) normFaceScore = 0.0;
      setIdentity(kf_tracker_.measurementNoiseCov, cv::Scalar_<float>::all(normFaceScore));
      measurement_.at<float>(0) = r.x;
      measurement_.at<float>(1) = r.y;
      measurement_.at<float>(2) = r.width;
      measurement_.at<float>(3) = r.height;
      kf_tracker_.correct(measurement_);

      // We see a face
      updateFaceHist = true;
    }
    else
    {
      kf_tracker_.statePost = kf_tracker_.statePre;
      kf_tracker_.errorCovPost = kf_tracker_.errorCovPre;
    }

    // TODO: MOVE THIS
    for (unsigned int k = 0; k < faces_.size(); k++)
    {
      rectangle(frame_debug_, faces_.at(k), CV_RGB(128, 128, 128));
    }

    beleif_.x = std::max<int>(kf_tracker_.statePost.at<float>(0), 0);
    beleif_.y = std::max<int>(kf_tracker_.statePost.at<float>(1), 0);
    beleif_.width = std::min<int>(kf_tracker_.statePost.at<float>(4), image_width_ - beleif_.x);
    beleif_.height = std::min<int>(kf_tracker_.statePost.at<float>(5), image_height_ - beleif_.y);

    cv::Point bel_center;
    bel_center.x = beleif_.x + (beleif_.width * 0.5);
    bel_center.y = beleif_.y + (beleif_.height * 0.5);

    if ((debug_level_ & 0x02) == 0x02)
    {
      txtstr.str("");
//      txtstr << "P:" << std::setprecision(3) << faceUncPos << " S:" << beleif.width << "x" << beleif.height;
//      cv::putText(debugFrame, txtstr.str(), belCenter + Point(0, 50), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(255,0,0));

      cv::rectangle(frame_debug_, beleif_, CV_RGB(0, 255, 255), 2);
//      cv::circle(frame_debug_, belCenter, belRad, CV_RGB(255,0,0));
//      cv::circle(frame_debug_, belCenter, (belRad - faceUncPos < 0) ? 0 : (belRad - faceUncPos), CV_RGB(255,255,0));
//      cv::circle(frame_debug_, belCenter, belRad + faceUncPos, CV_RGB(255,0,255));
    }

    search_roi_.x = std::max<int>(bel_center.x - kf_tracker_.statePost.at<float>(4) * 2, 0);
    search_roi_.y = std::max<int>(bel_center.y - kf_tracker_.statePost.at<float>(5) * 2, 0);
    int x2 = std::min<int>(bel_center.x + kf_tracker_.statePost.at<float>(4) * 2, image_width_);
    int y2 = std::min<int>(bel_center.y + kf_tracker_.statePost.at<float>(4) * 2, image_height_);
    search_roi_.width = x2 - search_roi_.x;
    search_roi_.height = y2 - search_roi_.y;


    if ((updateFaceHist) && (skin_enabled_))
    {
      //updateFaceHist is true when we see a real face (not all the times)
      cv::Rect samplingWindow;
//            samplingWindow.x = beleif.x + (0.25 * beleif.width);
//            samplingWindow.y = beleif.y + (0.1 * beleif.height);
//            samplingWindow.width = beleif.width * 0.5;
//            samplingWindow.height = beleif.height * 0.9;
      samplingWindow.x = measurement_.at<float>(0) + (0.25 * measurement_.at<float>(2));
      samplingWindow.y = measurement_.at<float>(1) + (0.10 * measurement_.at<float>(3));
      samplingWindow.width = measurement_.at<float>(2) * 0.5;
      samplingWindow.height = measurement_.at<float>(3) * 0.9;
      if ((debug_level_ & 0x04) == 0x04)
      {
        rectangle(frame_debug_, samplingWindow, CV_RGB(255, 0, 0));
      }
      cv::Mat _face = frame_raw_(samplingWindow);
      GenerateRegionHistogram(_face, face_hist_);
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

void CHumanTracker::TrackSkin()
{

  bool perform =
    (skin_enabled_) &&
    (
      (tracking_state_ == STATE_TRACK) ||
      (tracking_state_ == STATE_REJECT)
    );

  if (perform)
  {
    HistFilter(frame_raw_, skin_, skin_prior_, face_hist_, false);
    const double sum_skin = sum(skin_)[0];
    skin_ *= (1.0 / sum_skin);
    skin_prior_ = skin_.clone();
    if ((debug_level_ & 0x04) == 0x04)
    {
      cv::Mat visSkin;
      cv::normalize(skin_, visSkin, 1.0, 0.0, cv::NORM_MINMAX);
      visSkin.convertTo(frame_skin_, CV_8UC1, 255.0, 0.0);
    }
  }
}

void CHumanTracker::CalcOpticalFlow()
{
  static bool first = true;
  static bool firstCancel = true;
  static cv::Rect prevFaceRect;

  bool perform =
    (gesture_enabled_) &&
    (
      (tracking_state_ == STATE_TRACK) ||
      (tracking_state_ == STATE_REJECT)
    );

  if (!perform) return;

  cv::Mat rawFrameResized;
  const double fx = 0.5;
  const double fy = 0.5;
  cv::resize(frame_raw_, rawFrameResized, cv::Size(), fx, fy,  cv::INTER_LINEAR);

  if (first)
  {
    first = false;
    cvtColor(rawFrameResized, frame_raw_gray_prev_, CV_BGR2GRAY);
    return;
  }

  cv::Point belCenter;
  belCenter.x = (fx * beleif_.x) + (fx * beleif_.width * 0.5);
  belCenter.y = (fy * beleif_.y) + (fy * beleif_.height * 0.5);

  //‌ FlowRoi is now not being used to filter out any region, it is a placeholder
  // changed by Jake (745083b132aa4bfd8f77cb7c7aa1453348535920)
  // changed back by to hri_in_the_sky values by mani
  flow_roi_.x = std::max<int>(belCenter.x - fx * kf_tracker_.statePost.at<float>(4) * 3.0, 0);
  flow_roi_.y = std::max<int>(belCenter.y - fy * kf_tracker_.statePost.at<float>(5) * 3.0, 0);
  int x2 = std::min<int>(belCenter.x + fx * kf_tracker_.statePost.at<float>(4) * 3, image_width_);
  int y2 = std::min<int>(belCenter.y + fy * kf_tracker_.statePost.at<float>(4) * 0, image_height_);
  flow_roi_.width = x2 - flow_roi_.x;
  flow_roi_.height = y2 - flow_roi_.y;

  cvtColor(rawFrameResized, frame_raw_gray_, CV_BGR2GRAY);

  // TODO: Optimization here
  cv::Mat _i1 = frame_raw_gray_prev_;//(flowROI);
  cv::Mat _i2 = frame_raw_gray_;//(flowROI);

  int flags = 0;//OPTFLOW_USE_INITIAL_FLOW;
  if (firstCancel)
  {
    flags = 0;
    firstCancel = false;
  }
  cv::calcOpticalFlowFarneback(_i1, _i2 , flow_, 0.5, 3, 10, 3, 9, 1.9, flags); //OPTFLOW_USE_INITIAL_FLOW);
  std::vector<cv::Mat> flowChannels;
  cv::split(flow_, flowChannels);

  // This mask is very important because zero-valued elements
  // Should not be taken into account
  cv::Mat maskX;
  cv::Mat maskY;
  cv::Mat maskFull;

  maskX = abs(flowChannels[0]) > 0.01;
  maskY = abs(flowChannels[1]) > 0.01;
  bitwise_and(maskX, maskY, maskFull);

  // Gesture regions update
  gesture_region_[REG_TOPLEFT].x = flow_roi_.x;
  gesture_region_[REG_TOPLEFT].y = flow_roi_.y + (0.5 * flow_roi_.height);
  gesture_region_[REG_TOPLEFT].width = 0.5 * (flow_roi_.width - (fx * beleif_.width));
  gesture_region_[REG_TOPLEFT].height = flow_roi_.height;

  // changed by Jake (745083b132aa4bfd8f77cb7c7aa1453348535920)
  // changed back by to hri_in_the_sky values by mani
  gesture_region_[REG_TOPRIGHT].x = gesture_region_[REG_TOPLEFT].x + gesture_region_[REG_TOPLEFT].width + (fx * beleif_.width);
//  int _d = belCenter.x - (gesture_region_[REG_TOPLEFT].x + gesture_region_[REG_TOPLEFT].width);
//  gesture_region_[REG_TOPRIGHT].x = belCenter.x + _d;
  gesture_region_[REG_TOPRIGHT].y = gesture_region_[REG_TOPLEFT].y;
  gesture_region_[REG_TOPRIGHT].width = gesture_region_[REG_TOPLEFT].width;
  gesture_region_[REG_TOPRIGHT].height = gesture_region_[REG_TOPLEFT].height;

  // Safety check to downsampled image size
  SafeRectToImage(gesture_region_[REG_TOPLEFT], fx, fy);
  SafeRectToImage(gesture_region_[REG_TOPRIGHT], fx, fy);

  // Cancel Ego Motion & Face bias As much As possible
  int _row = 0;
  int _col = 0;
  std::vector<cv::Point2f> prev;
  std::vector<cv::Point2f> curr;
  std::vector<cv::Point2f> prev_stab;
  cv::Mat homography = cv::Mat::eye(3, 3, CV_32F);

  magnitude(flowChannels[0], flowChannels[1], flow_mag_);

//    ROS_INFO("Sum Mag Before: %6.f", sum(flowMag)[0]);
  if (stab_ == STABLIZE_MEDIAN)
  {

//    // The possible hand regions are not sampled for flow compenstation
//        // Initially we use inverted mask here
    cv::Mat maskRegions = cv::Mat::ones(maskX.rows, maskX.cols, CV_8UC1);
    cv::Mat maskAugmentedX = cv::Mat::ones(maskX.rows, maskX.cols, CV_8UC1);;
    cv::Mat maskAugmentedY = cv::Mat::ones(maskY.rows, maskY.cols, CV_8UC1);;

    cv::rectangle(maskRegions, gesture_region_[REG_TOPLEFT].tl(), gesture_region_[REG_TOPLEFT].br(), 0, CV_FILLED);
    cv::rectangle(maskRegions, gesture_region_[REG_TOPRIGHT].tl(), gesture_region_[REG_TOPRIGHT].br(), 0, CV_FILLED);

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
        medX = CalcMedian(flowChannels[0], 25, minX, maxX, maskAugmentedX);
      }
      catch (cv::Exception &e)
      {
        ROS_WARN("Calculate median failed for X flow with %s", e.what());
      }

      try
      {
        medY = CalcMedian(flowChannels[1], 25, minY, maxY, maskAugmentedY);
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
      cv::add(flowChannels[0], cv::Scalar::all(-medX), flowChannels[0], maskX);
      cv::add(flowChannels[1], cv::Scalar::all(-medY), flowChannels[1], maskY);
    }
//        // Face Bias
    if (tracking_state_ == STATE_TRACK)
    {
      cv::Rect fROI = faces_.at(0);
      fROI.x *= fx;
      fROI.y *= fy;
      fROI.width *= fx;
      fROI.height *= fy;
      SafeRectToImage(fROI, fx, fy);

      cv::Mat faceFlowWindowX = flowChannels[0](fROI);
      cv::Mat faceFlowWindowY = flowChannels[1](fROI);
      cv::Mat faceMaskX = maskX(fROI);
      cv::Mat faceMaskY = maskY(fROI);

      cv::minMaxLoc(faceFlowWindowX, &minX, &maxX, 0, 0, faceMaskX);
      cv::minMaxLoc(faceFlowWindowY, &minY, &maxY, 0, 0, faceMaskY);
      if ((maxX > minX) && (maxY > minY))
      {
        try
        {
          medX = CalcMedian(faceFlowWindowX, 25, minX, maxX, faceMaskX);
          medY = CalcMedian(faceFlowWindowY, 25, minY, maxY, faceMaskY);

          // Now we want to cancel motion only in the interesting regions
          maskRegions = 1 - maskRegions;
          bitwise_and(maskX, maskRegions, maskAugmentedX);
          bitwise_and(maskY, maskRegions, maskAugmentedY);

          cv::add(flowChannels[0], cv::Scalar::all(-medX), flowChannels[0], maskAugmentedX);
          cv::add(flowChannels[1], cv::Scalar::all(-medY), flowChannels[1], maskAugmentedY);
        }
        catch (cv::Exception &e)
        {
          ROS_WARN("Face flow cancellation failed with %s", e.what());
        }
      }
    }
  }
  else if (stab_ == STABLIZE_HOMOGRAPHY)
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

    for (_row = 0; _row < flow_.rows; _row += 1)
    {
      for (_col = 0; _col < flow_.cols; _col += 1)
      {
        if (maskFull.at<uchar>(_row, _col) == 0) continue;
        const cv::Point2f& prev_point = (1.0 / fx) * (cv::Point2f(_col, _row) - flow_.at<cv::Point2f>(_row, _col));
        const cv::Point2f& curr_point = (1.0 / fx) * cv::Point2f(_col, _row);
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
        const cv::Point2f& flow_extra = curr[i] - prev_stab[i];
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
  magnitude(flowChannels[0], flowChannels[1], flow_mag_);


  if (skin_enabled_)
  {
    cv::Mat skin_small;
    cv::resize(skin_, skin_small, cv::Size(0, 0), fx, fy);
    cv::normalize(skin_small, skin_small, 1.0, 0.0, cv::NORM_MINMAX);
    cv::Mat skin_mask = skin_small > 0.7;
    cv::multiply(flow_mag_, skin_mask, flow_mag_, 1.0, CV_32FC1);
  }
  else
  {
    ;//threshold(flowMag, flowMag, (double) minFlow, 0.0, THRESH_TOZERO);
  }
//    ROS_INFO("Sum Mag After: %6.f", sum(flowMag)[0]);



  // Big Question: Why does the below line make the signal so weak?
//  normalize(flowMag, flowMag, 0.0, 1.0, NORM_MINMAX);

  cv::Rect r;

  //std::swap(prevRawFrameGray, rawFramGray);
  frame_raw_gray_prev_ = frame_raw_gray_.clone();

  for (int i = 0; i < 2; i++)
  {

    cv::Rect reg = gesture_region_[i];
    reg.x = gesture_region_[i].x;// - flowROI.x;
    reg.y = gesture_region_[i].y;// - flowROI.y;

    float sFlow = 0.0;
    if (skin_enabled_)
    {
      sFlow = ((gesture_region_[i].width > 0) && (gesture_region_[i].height > 0)) ?
              mean(flow_mag_(reg), maskFull(reg))[0] : 0.0;
    }
    else
    {
      // Threshold and mean over thresholded values
      sFlow = ((gesture_region_[i].width > 0) && (gesture_region_[i].height > 0)) ?
              mean(flow_mag_(reg), (flow_mag_(reg) > min_flow_))[0] : 0.0;
    }


    //flowScoreInRegion[i] = (0.5 * flowScoreInRegion[i]) + (0.5 * sFlow);
    // No filtering should be done here
    flow_score_in_region_[i] = sFlow;
  }

  // Visulization
  if ((debug_level_ & 0x10) == 0x10)
  {
    std::vector<cv::Mat> channels;
    split(rawFrameResized, channels);

    // HSV Color Space
    // Red to blue specterum is between 0->120 degrees (Red:0, Blue:120)
    flow_mag_.convertTo(channels[0], CV_8UC1, 10);
    channels[0] = cv::Scalar::all(255) - channels[0];
    //flowMag = 120 - flowMag;
    channels[1] = cv::Scalar::all(255.0);
    channels[2] = _i2;//Scalar::all(255.0);
    merge(channels, frame_optical_);
    cvtColor(frame_optical_, frame_optical_, CV_HSV2BGR);
    if (false) //(stablization == STABLIZE_HOMOGRAPHY) {
    {
      for (unsigned int i = 0; i < prev.size(); i++)
      {
        line(frame_debug_, prev[i], curr[i], CV_RGB(0, 255, 0));
        //            line(debugFrame, prev_stab[i], curr[i], CV_RGB(255,0,0));
        //            circle(opticalFrame, curr[i], 1, CV_RGB(255,255,255));
      }
    }
  }

  if ((debug_level_ & 0x02) == 0x02)
  {
    if (stab_ == STABLIZE_HOMOGRAPHY)
      cv::warpPerspective(frame_debug_, frame_debug_, homography, cv::Size(frame_debug_.cols, frame_debug_.rows), cv::WARP_INVERSE_MAP);
    //cv::warpAffine(debugFrame, debugFrame, homography(Rect(0,0,3,2)), Size(debugFrame.cols, debugFrame.rows), cv::WARP_INVERSE_MAP);

    //rectangle(debugFrame, Rect(flowROI.x / fx, flowROI.y / fy, flowROI.width / fx, flowROI.height / fy), CV_RGB(255, 0, 0));
    for (int i = 0; i < 2; i++)
    {
      // Only for visualization
      gesture_region_[i].x /= fx;
      gesture_region_[i].y /= fy;
      gesture_region_[i].width /= fx;
      gesture_region_[i].height /= fy;

      std::stringstream txtstr;
      rectangle(frame_debug_, gesture_region_[i], CV_RGB(0, 0, 0));
      cv::Point2d center;
      center.x = gesture_region_[i].x + gesture_region_[i].width / 2;
      center.y = gesture_region_[i].y + gesture_region_[i].height / 2;
      txtstr << std::fixed << std::setprecision(3) << flow_score_in_region_[i];
//      putText(debugFrame, txtstr.str(), center, FONT_HERSHEY_PLAIN, 1, CV_RGB(0,0,255));
    }
  }



}

void CHumanTracker::Draw()
{
  return ;
}

void CHumanTracker::EnableCallback(const std_msgs::BoolConstPtr & enable)
{
  const bool& en = enable->data;

  ROS_INFO_STREAM("[BEH] Human Request: " << (en ? "Enable" : "Disable"));

  if (!is_enabled_ && en)
  {
    Reset();
    is_enabled_ = true;
  }

  if (is_enabled_ && !en)
  {
    Reset();
    is_enabled_ = false;
  }
}

void CHumanTracker::VisionCallback(const sensor_msgs::ImageConstPtr& frame)
{
  t_start_ = ros::Time::now();
  if (is_enabled_)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      frame_id_ = frame->header.frame_id;
      should_publish_ = true;
      cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[HUM] Error converting the input image: %s", e.what());
      return;
    }

    if (is_inited_ == false)
    {
      // Get Image Info from the first frame
      is_inited_ = true;
      image_width_ = cv_ptr->image.cols;
      image_height_ = cv_ptr->image.rows;
      ROS_INFO("[HUM] Image size is %d x %d", image_width_, image_height_);
      Reset();
    }

    //TODO: Check clone
    this->frame_raw_ = cv_ptr->image;

    t_face_start_ = ros::Time::now();
    DetectAndTrackFace();

    t_skin_start_ = ros::Time::now();
    TrackSkin();

    t_flow_start_ = ros::Time::now();
    CalcOpticalFlow();

    Publish();
  }

  t_end_ = ros::Time::now();

}

template<class T>
inline void get_param(const ros::NodeHandle& nh,
                      const std::string& param_name, T& var, const T default_value)
{
  nh.param(param_name, var, default_value);
  ROS_INFO_STREAM("[HUM] Param " << param_name << " : " << var);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomy_human");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  image_transport::ImageTransport it(n);

  std::string p_xmlFile, p_xmlFileProfile;

  if (false == ros::param::get("~cascade_file", p_xmlFile))
  {
    ROS_FATAL("[HUM] No cascade file provided, use `cascade_file` param to set it.");
    ros::shutdown();
    exit(1);
  }
  else
  {
    ROS_INFO("[HUM] Cascade file: %s", p_xmlFile.c_str());
  }

  bool p_profileFaceEnabled;
  ros::param::param("~profile_hack_enabled", p_profileFaceEnabled, false);
  ROS_INFO("[HUM] Profile Face Hack is %s", p_profileFaceEnabled ? "Enabled" : "Disabled");

  if ((false == ros::param::get("~cascade_profile_file", p_xmlFileProfile)) && (p_profileFaceEnabled))
  {
    ROS_FATAL("[HUM] No profile cascade file provided, use `cascade_profile_file` param to set it.");
    ros::shutdown();
    exit(2);
  }
  else
  {
    ROS_INFO("[HUM] Profile Cascade file: %s", p_xmlFileProfile.c_str());
  }

  bool p_skinEnabled;
  bool p_gestureEnabled;
  int p_stablization;
  int  p_debugMode;
  int p_initialScoreMin, p_initialDetectFrames, p_initialRejectFrames, p_minFlow;
  int p_minFaceSizeW, p_minFaceSizeH, p_maxFaceSizeW, p_maxFaceSizeH;
  double p_mCov, p_pCov;
  bool p_start_paused;

  get_param<bool>(n_priv, "gesture_enabled", p_gestureEnabled, false);
  get_param<bool>(n_priv, "skin_enabled", p_skinEnabled, false);
  get_param<int>(n_priv, "flowstablize_mode", p_stablization, 1);
  get_param<int>(n_priv, "debug_mode", p_debugMode, 0x02);
  get_param<int>(n_priv, "initial_min_score", p_initialScoreMin, 5);
  get_param<int>(n_priv, "initial_detect_frames", p_initialDetectFrames, 6);
  get_param<int>(n_priv, "initial_reject_frames", p_initialRejectFrames, 6);
  get_param<int>(n_priv, "min_flow", p_minFlow, 10);
  get_param<int>(n_priv, "min_face_width", p_minFaceSizeW, 12);
  get_param<int>(n_priv, "min_face_height", p_minFaceSizeH, 18);
  get_param<int>(n_priv, "max_face_width", p_maxFaceSizeW, 60);
  get_param<int>(n_priv, "max_face_height", p_maxFaceSizeH, 80);
  get_param<double>(n_priv, "meas_cov", p_mCov, 1.0);
  get_param<double>(n_priv, "proc_cov", p_pCov, 0.05);
  get_param<bool>(n_priv, "start_paused", p_start_paused, false);

  /**
   * The queue size seems to be very important in this project
   * If we can not complete the calculation in the 1/fps time slot, we either
   * drop the packet (low queue size) or process all frames which will lead to
   * additive delays.
   */

  ros::Publisher facePub = n.advertise<autonomy_human::human>("human/human", 5);
  ros::Publisher allDetectionsPub = n.advertise<autonomy_human::raw_detections>("human/raw_detections", 5);
  image_transport::Publisher debugPub = it.advertise("human/debug/image_raw", 1);
  image_transport::Publisher skinPub = it.advertise("human/skin/image_raw", 1);
  image_transport::Publisher opticalPub = it.advertise("human/optical/image_raw", 1);

  CHumanTracker human_tracker(p_start_paused, p_xmlFile, p_xmlFileProfile,
                              p_pCov, p_mCov,
                              p_minFaceSizeW, p_minFaceSizeH, p_maxFaceSizeW, p_maxFaceSizeH,
                              p_initialScoreMin, p_initialDetectFrames, p_initialRejectFrames, p_minFlow,
                              p_profileFaceEnabled, p_skinEnabled, p_gestureEnabled,
                              p_debugMode, p_stablization,
                              facePub, allDetectionsPub, debugPub, skinPub, opticalPub);

  ros::Subscriber enableSub = n.subscribe("human/enable", 10, &CHumanTracker::EnableCallback, &human_tracker);
  image_transport::Subscriber visionSub = it.subscribe("camera/image_raw", 10, &CHumanTracker::VisionCallback, &human_tracker);

  ROS_INFO("[HUM] Starting Autonomy Human ...");
  if (p_start_paused)
  {
    ROS_WARN("[HUM] Starting paused!");
  }

  ros::spin();

  return 0;
}

