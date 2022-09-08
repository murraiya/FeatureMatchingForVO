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

vector<cv::KeyPoint> TargetKeypoints, ReferenceKeypoints;
cv::Mat TargetDescriptor, ReferDescriptor;

cv::Ptr<cv::Feature2D> orb = cv::ORB::create(10);
cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(500);
cv::Ptr<cv::Feature2D> brisk = cv::BRISK::create();

cv::Ptr<cv::DescriptorMatcher> Matcher_ORB = cv::BFMatcher::create(cv::NORM_HAMMING);		// Brute-Force matcher create method
cv::Ptr<cv::DescriptorMatcher> Matcher_BRISK = cv::BFMatcher::create(cv::NORM_HAMMING);		// Brute-Force matcher create method
cv::Ptr<cv::DescriptorMatcher> Matcher_SIFT = cv::BFMatcher::create(cv::NORM_L2);			// Brute-Force matcher create method

vector<cv::DMatch> matches;	// Class for matching keypoint descriptors.
int cnt = 0;
void processing(cv::Mat&);


void ImgSubCallback(const sensor_msgs::Image raw_img){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_img, sensor_msgs::image_encodings::BGR8);
    processing(cv_ptr->image);
}


void processing(cv::Mat& frame){
    // 종료 시까지 영상 출력
    
    if(frame.empty())
        cout<<"NO IMG NO IMG NO IMG"<<endl;

    //imshow("ORB Video", frame);		// 현재 웹캠을 통해 실시간으로 받고 있는 영상을 보여주기
    int key = cv::waitKey(1);
    // cout<<"key : "<<key<<endl;
    cnt++;

    if(cnt==4700)
    {
        cout << "References image으로 선택됩니다.\n\n";
        Refer_image = frame; // 눌렀을 때의 해당 프레임을 Reference frame으로 선택
        cv::cvtColor(Refer_image, Refer_gray_image, cv::COLOR_RGB2GRAY);

        // imshow("Reference Image", Refer_image);			// References image 보여주기
        // imshow("Reference Gray Image", Refer_gray_image);
    }

    if(cnt==4750)
    // ORB를 이용, Target image 선정
    {
        // References image에 대해 feature 뽑기
        orb->detectAndCompute(Refer_gray_image, cv::Mat(), ReferenceKeypoints, ReferDescriptor);// detects keypoints and computes the descriptors

        // Refer image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
        // cv::Mat sift_refer_result;
        // cv::drawKeypoints(Refer_gray_image, ReferenceKeypoints, sift_refer_result);
        // imshow("Result_sift_refer_result", sift_refer_result);
        cout << "ORB를 선택하셨습니다. 해당 이미지를 Target image으로 선택합니다.\n\n";
        Target_image = frame; // 눌렀을 때의 해당 프레임을 Target image으로 선택
        cv::cvtColor(Target_image, Target_gray_image, cv::COLOR_RGB2GRAY);

        // imshow("Target Image", Target_image);	// Target image 보여주기
        // imshow("Target Gray Image", Target_gray_image);

        

        // Target image에 대해 feature 뽑기
        orb->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);// detects keypoints and computes the descriptors

     

        // Refer image와 Target image의 fature를 이용하여 Feature matching 실행
        Matcher_ORB->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.

        sort(matches.begin(), matches.end());
        const int match_size = matches.size();
        vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.5f));

        cv::Mat Result_ORB;
        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, matches, Result_ORB, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.
       

        imshow("Result_ORB", Result_ORB);
        imwrite("/home/autonav/feature_ws/Matching_ORB.jpg", Result_ORB);

        cv::waitKey(0);
    }
}

int main(int argc, char** argv) {
    cout<<"! ORB !"<<endl;


    ros::init(argc, argv, "ORB_feature_matching_node");
    ros::NodeHandle nh;
    
    ros::Subscriber raw_image_sub = nh.subscribe<sensor_msgs::Image>("/raw_image", 1, ImgSubCallback);
    
    if(ros::ok())
        ros::spin();


    return 0;
}
