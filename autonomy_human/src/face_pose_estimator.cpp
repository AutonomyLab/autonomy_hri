#include "face_pose_estimator.hpp"

#include <opencv2/opencv.hpp>								// cv::Mat and related junk
using namespace cv;

#include <dlib/image_processing/full_object_detection.h>	// for dlib::full_object_detection object
#include <dlib/image_processing/shape_predictor.h>			// for dlib::shape_predictor object
#include <dlib/opencv.h>									// for dlib::cv_image and related stuff
// I'm intentionally NOT using the dlib namespace for clarity, because dlib
// uses some odd names that are not immediately obvious to me...anyone familiar
// with dlib might find this annoying; deal with it.

#include <string>
#include <sstream>
#include <iostream>
using namespace std;

FacePoseEstimator::FacePoseEstimator(string &landmarksFile)
{
	// we require a face landmarking model file in order to do face detection;
	// it should be located in the same directory as the executable, and the
	// name of the file should be stored in landmarksFile
	try
	{
		dlib::deserialize(landmarksFile) >> poseModel;
	}
	catch(dlib::serialization_error& e)
    {
		cout << "The file '" << landmarksFile << "' could not be found!" << endl;
        cout << "This is dlib's default face landmarking model file; it must be in the same directory as this executable!" << endl;
        cout << "You can get it from the following URL:" << endl;
        cout << "\thttp://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
    }
    catch(exception &e)
    {
		// uh-oh, something else unrelated happened
		cout << "Some interesting exception occurred:" << endl;
        cout << e.what() << endl;
    }

	// expect an initial sample to be needed before we can filtering
	initialSample = true;

	// build our reference points---these correspond to vertices from a 3D model
    referencePoints.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));              // nose tip
    referencePoints.push_back(cv::Point3d(0.0f, -330.0f, 65.0f));          // chin
    referencePoints.push_back(cv::Point3d(-200.0f, 170.0f, 135.0f));       // left eye left corner
    referencePoints.push_back(cv::Point3d(200.0f, 170.0f, 135.0f));        // right eye right corner
    referencePoints.push_back(cv::Point3d(-150.0f, -150.0f, 125.0f));      // left mouth corner
    referencePoints.push_back(cv::Point3d(150.0f, -150.0f, 125.0f));       // right mouth corner
}

FacePoseEstimator::~FacePoseEstimator()
{
	// no dynamically-allocated members to free
}

void FacePoseEstimator::getLandmarkFeatures(Mat &img, Mat &debug, const Rect &roi, dlib::full_object_detection &faceDetection)
{
	dlib::cv_image<dlib::bgr_pixel> dlibImg;			// we need to convert the cv::Mat into something dlib can use
	dlib::rectangle dlibROI;							// we also need to convert the CV-style ROI into dlib's rectangle object

	// convert incoming image into dlib format (trivial shallow copy; should be negligible performance impact)
	dlibImg = dlib::cv_image<dlib::bgr_pixel>(img);

	// convert incoming ROI into dlib format (again, easy)
	dlibROI = dlib::rectangle(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);

	// now that we have everything in the right format, perform dlib's face pose estimation
	// (note: syntactical oddity---poseModel is our dlib::shape_predictor object member)
	faceDetection = poseModel(dlibImg, dlibROI);

	// uncomment to render numbered indices of each vertex in the detected face object
	// (useful for dev purposes when we want to know which vertices are relevant)
	//showFacePointIndices(&debug, &faceDetection);
}

void FacePoseEstimator::estimatePoseRaw(Mat &img, Mat &debug, const Rect &roi, Mat &rotation)
{
	dlib::full_object_detection faceDetection;			// dlib's representation of detected face points

	// do landmark detection
	getLandmarkFeatures(img, debug, roi, faceDetection);

	// use OpenCV's solvePNP() to estimate the pose of the face
	estimatePoseFromReference(img, debug, faceDetection, rotation, false);
}

void FacePoseEstimator::estimatePoseFiltered(Mat &img, Mat &debug, const Rect &roi, Mat &rotation)
{
	dlib::full_object_detection faceDetection;			// dlib's representation of detected face points

	// do landmark detection
	getLandmarkFeatures(img, debug, roi, faceDetection);

	// use OpenCV's solvePNP() to estimate the pose of the face
	estimatePoseFromReference(img, debug, faceDetection, rotation, true);
}

// face pose estimation based on code from
// http://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
// basically, this uses reference points from a 3D model to determine where the
// detected 2D face points lie; uses OpenCV's solvePNP() function to do this
void FacePoseEstimator::estimatePoseFromReference(Mat &img, Mat &debug, dlib::full_object_detection &face, Mat &rotation, bool useFilter)
{
	// for our low-pass exponential filter
	const float ALPHA = 0.6;

	// relevant indices of face points in face.part([index]); determined using showFacePointIndices()
    const int NOSE_TIP = 30;					// nose tip
    const int CHIN = 8;							// chin tip
    const int LEFT_EYE = 45;                    // left eye left corner
    const int RIGHT_EYE = 36;                   // right eye right corner
    const int LEFT_MOUTH = 54;                  // left mouth corner
    const int RIGHT_MOUTH = 48;                 // right mouth corner

    std::vector<cv::Point2d> image_points;      // actual face points as detected

	Mat translation;							// not actually used; required by solvePNP()

	// massage our data a bit---convert from dlib::vector<double,2> to cv::Point2d
	cv::Point2d newNose(face.part(NOSE_TIP).x(), face.part(NOSE_TIP).y());
	cv::Point2d newChin(face.part(CHIN).x(), face.part(CHIN).y());
	cv::Point2d newLeftEye(face.part(LEFT_EYE).x(), face.part(LEFT_EYE).y());
	cv::Point2d newRightEye(face.part(RIGHT_EYE).x(), face.part(RIGHT_EYE).y());
	cv::Point2d newLeftMouth(face.part(LEFT_MOUTH).x(), face.part(LEFT_MOUTH).y());
	cv::Point2d newRightMouth(face.part(RIGHT_MOUTH).x(), face.part(RIGHT_MOUTH).y());

	// do we use previous result to filter, or just take the raw points?
	if(initialSample || !useFilter)
	{
		// record initial filtering sample as taken
		if(initialSample)
		{
			initialSample = false;
		}

		nose = newNose;
		chin = newChin;
		leftEye = newLeftEye;
		rightEye = newRightEye;
		leftMouth = newLeftMouth;
		rightMouth = newRightMouth;
	}
	else
	{
		// simple low-pass filter
		nose = nose * ALPHA + ((1.0 - ALPHA) * newNose);
		chin = chin * ALPHA + ((1.0 - ALPHA) * newChin);
		leftEye = leftEye * ALPHA + ((1.0 - ALPHA) * newLeftEye);
		rightEye = rightEye * ALPHA + ((1.0 - ALPHA) * newRightEye);
		leftMouth = leftMouth * ALPHA + ((1.0 - ALPHA) * newLeftMouth);
		rightMouth = rightMouth * ALPHA + ((1.0 - ALPHA) * newRightMouth);
	}

    // store the locations of the relevant feature points
    image_points.push_back(nose);         		// nose tip
    image_points.push_back(chin);            	// chin
    image_points.push_back(leftEye);         	// left eye left corner
    image_points.push_back(rightEye);       	// right eye right corner
    image_points.push_back(leftMouth);			// left Mouth corner
    image_points.push_back(rightMouth);			// right mouth corner

    // camera internals
    double focal_length = img.cols; // Approximate focal length.
    cv::Point2d center = cv::Point2d(img.cols/2,img.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

    // Solve for pose
    cv::solvePnP(referencePoints, image_points, camera_matrix, dist_coeffs, rotation, translation);
	//cv::solvePnPRansac(referencePoints, image_points, camera_matrix, dist_coeffs, rotation, translation);

    // Project a 3D point (0, 0, 1000.0) onto the image plane.
    // We use this to draw a line sticking out of the nose
    std::vector<cv::Point3d> nose_end_point3D;
    std::vector<cv::Point2d> nose_end_point2D;
    nose_end_point3D.push_back(cv::Point3d(0,0,-1000.0));

	// OpenCV function for PNP
    projectPoints(nose_end_point3D, rotation, translation, camera_matrix, dist_coeffs, nose_end_point2D);

	// graphically show the points
    for(int i=0; i < image_points.size(); i++)
    {
        circle(debug, image_points[i], 3, cv::Scalar(0,0,255), -1);
    }

	// draw a line showing the direction of the detected face
    cv::line(debug,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
}

// simple debugging routine; simply overlays the given image (containing a face)
// with the zero-based indices corresponding to each point from the detected face
void FacePoseEstimator::showFacePointIndices(Mat *debug, dlib::full_object_detection *face)
{
    stringstream ss;                        // used to stringify the point index
    dlib::vector<double, 2> point;          // face point we convert to a cv::Point
    unsigned int i;

    // iterate through all face points
    for(i = 0; i < face -> num_parts(); ++i)
    {
        // stringify the index number
        ss.str("");
        ss << i;
        point = face -> part(i);

        // label the point with its index in face -> part(i)
        putText(*debug, ss.str(), cv::Point(point.x(), point.y()), cv::FONT_HERSHEY_SIMPLEX, 0.25, cvScalar(200,200,250), 1, CV_AA);
    }
}

void FacePoseEstimator::resetFilter()
{
	initialSample = true;
}
