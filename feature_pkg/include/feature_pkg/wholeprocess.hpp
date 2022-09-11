#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow
#include <vector>

#include <iostream>
#include <iomanip>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
 
using namespace cv;
using namespace std;


cv::Mat H;
std::vector<Point2f> beforeFramePt;
std::vector<Point2f> afterFramePt;

cv::Mat frame;
cv::Mat Refer_image;
cv::Mat Refer_gray_image;
cv::Mat Target_image;
cv::Mat Target_gray_image;

cv::VideoWriter videoWriter;

vector<cv::KeyPoint> TargetKeypoints, ReferenceKeypoints;
cv::Mat TargetDescriptor, ReferDescriptor;

cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(100);
cv::Ptr<cv::DescriptorMatcher> Matcher_SIFT = cv::BFMatcher::create(cv::NORM_L2, true);			// Brute-Force matcher create method

cv::Ptr<cv::Feature2D> orb = cv::ORB::create(100);
cv::Ptr<cv::DescriptorMatcher> Matcher_ORB = cv::BFMatcher::create(cv::NORM_HAMMING, true);			// Brute-Force matcher create method

vector<cv::DMatch> matches;	// Class for matching keypoint descriptors.



int gettingH(const vector<cv::DMatch>& good_matches);



