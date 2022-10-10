#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Transform.h>
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

char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;  
int x;
int y;
cv::Point textOrg(10, 50);
cv::Mat traj=cv::Mat(600, 600, CV_8UC3, Scalar(0));


double scale = 1.00;

geometry_msgs::Transform poseMessage;

cv::Mat E;
cv::Mat R, t;
cv::Mat R_f, t_f;

float data[]={
    1356,    0,      941,
    0,       1354,   597,
    0,       0,      1      };

cv::Mat intrinsic=cv::Mat(3, 3, CV_32F, data);

std::vector<Point2f> beforept;
std::vector<Point2f> afterpt;

int gettingPose(vector<cv::KeyPoint> keypoints_before, vector<cv::KeyPoint> keypoints_after, vector<cv::DMatch> good_matches);



