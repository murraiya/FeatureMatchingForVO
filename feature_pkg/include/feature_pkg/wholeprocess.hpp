#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
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

cv::Mat E;
cv::Mat R, t;

float data[]={
    1356,    0,      941,
    0,       1354,   597,
    0,       0,      1      };

cv::Mat intrinsic=cv::Mat(3, 3, CV_32F, data);

std::vector<Point2f> beforept;
std::vector<Point2f> afterpt;

int gettingPose(vector<cv::KeyPoint> keypoints_before, vector<cv::KeyPoint> keypoints_after, vector<cv::DMatch> good_matches);



