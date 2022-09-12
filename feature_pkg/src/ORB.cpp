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



using namespace std;


cv::Mat frame;
cv::Mat Refer_image;
cv::Mat Refer_gray_image;
cv::Mat Target_image;
cv::Mat Target_gray_image;

cv::VideoWriter videoWriter;


vector<cv::KeyPoint> TargetKeypoints, ReferenceKeypoints;
cv::Mat TargetDescriptor, ReferDescriptor;

cv::Ptr<cv::Feature2D> orb = cv::ORB::create(100);
cv::Ptr<cv::DescriptorMatcher> Matcher_ORB = cv::BFMatcher::create(cv::NORM_HAMMING, true);			// Brute-Force matcher create method

vector<cv::DMatch> matches;	// Class for matching keypoint descriptors.

void ImgSubCallback(const sensor_msgs::Image raw_img){
    cout<<"ORB: subscribed"<<endl;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_img, sensor_msgs::image_encodings::BGR8);
    
    if(cv_ptr->image.empty())
        cout<<"NO IMG NO IMG NO IMG"<<endl;

    Target_image = cv_ptr->image; 
    cv::cvtColor(Target_image, Target_gray_image, cv::COLOR_RGB2GRAY);
        
    if(Refer_gray_image.empty()==0)
    {

        // Refer image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
        // cv::Mat sift_refer_result;
        // cv::drawKeypoints(Refer_gray_image, ReferenceKeypoints, sift_refer_result);
        // imshow("Result_sift_refer_result", sift_refer_result);
        
        //feature 뽑기
        orb->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);// detects keypoints and computes the descriptors
        orb->detectAndCompute(Refer_gray_image, cv::Mat(), ReferenceKeypoints, ReferDescriptor);// detects keypoints and computes the descriptors

        // Refer image와 Target image의 fature를 이용하여 Feature matching 실행
        Matcher_ORB->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.

        sort(matches.begin(), matches.end());
        vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + 50);

        cv::Mat Result_ORB;
        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, good_matches, Result_ORB, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.

        // imshow("Result_ORB", Result_ORB);
        // cv::waitKey(1);

        if(Result_ORB.empty()==1){
        cout<<"match fail"<<endl;
        }
        videoWriter << Result_ORB;
        good_matches.clear();
    }

    Refer_gray_image=Target_gray_image.clone();
}



int main(int argc, char** argv) {
    cout<<"! ORB !"<<endl;

    videoWriter.open("/media/autonav/SJ_SSD/Matching_ORB.avi", cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 50, cv::Size(3840,1200), 1); //u can modify framerate

    ros::init(argc, argv, "ORB_feature_matching_node");
    ros::NodeHandle nh;
    
    ros::Subscriber raw_image_sub = nh.subscribe<sensor_msgs::Image>("/raw_image", 10, ImgSubCallback);
    
    if(ros::ok())
        ros::spin();


    return 0;
}
