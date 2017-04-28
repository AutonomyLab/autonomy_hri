// face_pose_estimator.hpp
// Geoff Nagy
// This class is basically the whole point of the project; the other source files
// are just to support the face-detection that we need in the first place. This
// class is responsible for detecting facial landmarks in a given ROI of an image,
// and making an estimation of the orientation of the face in 3D space.

#pragma once

#include <opencv2/opencv.hpp>								// for cv::Mat

#include <dlib/image_processing/full_object_detection.h>	// for dlib::full_object_detection
#include <dlib/image_processing/shape_predictor.h>			// for dlib::shape_predictor

#include <string>
#include <vector>

class FacePoseEstimator
{
private:
	// name of file in the executable directory dlib uses to detect face landmarks
	static const std::string SHAPE_PREDICTOR_LANDMARKS_FILE;

	// used to determine face landmarks
	dlib::shape_predictor poseModel;

	// reference of 3D facial points used for solvePNP()
	std::vector<cv::Point3d> referencePoints;

	// when using filtering, we expect one previous result to already be present
	bool initialSample;
	cv::Point2d nose;				// this are facial points from a previous
	cv::Point2d chin;				// call to estimatePoseFiltered(), and are
	cv::Point2d leftEye;			// needed to perform the filtered face
	cv::Point2d rightEye;			// pose estimation that works very smoothly
	cv::Point2d leftMouth;
	cv::Point2d rightMouth;

	// uses OpenCV's solvePNP() function to guess the pose of the facial landmarks given,
	// using known points from a 3D model as a reference
	void estimatePoseFromReference(cv::Mat &img, cv::Mat &debug, dlib::full_object_detection &face, cv::Mat &rotation, bool useFilter);

	// perform landmark detection
	void getLandmarkFeatures(cv::Mat &img, cv::Mat &debug, const cv::Rect &roi, dlib::full_object_detection &faceDetection);

	// dev function used to render the numbered indices of each facial landmark vertex
	void showFacePointIndices(cv::Mat *debug, dlib::full_object_detection *face);

public:
	FacePoseEstimator(std::string &landmarksFile);
	~FacePoseEstimator();

	// given an OpenCV image and an ROI known to contain a face, this method performs
	// face landmark detection and returns the estimated pose of the face as a 3D vector;
	// note that this contains no filtering; it is noisier than estimatePoseFiltered(),
	// but will work for any number of faces
	void estimatePoseRaw(cv::Mat &img, cv::Mat &debug, const cv::Rect &roi, cv::Mat &rotation);

	// given an OpenCV image and an ROI known to contain a face, this method performs
	// face landmark detection and returns the estimated pose of the face as a 3D vector
	// AFTER being filtered through a low-pass exponential filter---this is quite
	// smooth, but assumes that this method is only called once per frame, and that
	// we're tracking the same face relatively consistently across frames
	void estimatePoseFiltered(cv::Mat &img, cv::Mat &debug, const cv::Rect &roi, cv::Mat &rotation);

	// if we lose a face detection, it's probably a good idea to reset the filtering,
	// in case things move around a bit
	void resetFilter();
};
