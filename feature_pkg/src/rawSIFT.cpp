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

cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(50);
cv::Ptr<cv::DescriptorMatcher> Matcher_SIFT = cv::BFMatcher::create(cv::NORM_L2);			// Brute-Force matcher create method

vector<cv::DMatch> matches;	// Class for matching keypoint descriptors.

void ImgSubCallback(const sensor_msgs::Image raw_img){
    cout<<"SIFT: subscribed"<<endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_img, sensor_msgs::image_encodings::BGR8);
    
    if(cv_ptr->image.empty())
        cout<<"NO IMG NO IMG NO IMG"<<endl;

    // Refer image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
    /*cv::Mat sift_refer_result;
    cv::drawKeypoints(Refer_gray_image, ReferenceKeypoints, sift_refer_result);
    imshow("Result_sift_)resfer_result", sift_refer_result);*/

    Target_image = cv_ptr->image; 
    cv::cvtColor(Target_image, Target_gray_image, cv::COLOR_RGB2GRAY);
    
    if(Refer_gray_image.empty()==0) //ref 이미지 (전 이미지) 있으면 매칭 진행
    {
        sift->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);
        sift->detectAndCompute(Refer_gray_image, cv::Mat(), ReferenceKeypoints, ReferDescriptor);

        // Target image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
        /*cv::Mat sift_target_result;
        cv::drawKeypoints(Target_gray_image, TargetKeypoints, sift_target_result);
        imshow("Result_sift_Target_result", sift_target_result);*/


        // Refer image와 Target image의 fature를 이용하여 Feature matching 실행

        Matcher_SIFT->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.

        cv::Mat Result_SIFT;
        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, matches, Result_SIFT, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.

        // imshow("Result_SIFT", Result_SIFT);
        // cv::waitKey(1);


        if(Result_SIFT.empty()==1){
            cout<<"match fail"<<endl;
        }

        videoWriter << Result_SIFT;

    }

    Refer_gray_image=Target_gray_image.clone();
    cout<<"SIFT: subscribed_FIN"<<endl;
}

int main(int argc, char** argv) {
    cout<<"! SIFT !"<<endl;

    videoWriter.open("/media/autonav/SJ_SSD/Matching_SIFT_raw_itisnotforpaper.avi", cv::VideoWriter::fourcc('M', 'P', 'E', 'G'), 50, cv::Size(3840,1200), 1); //u can modify framerate

    ros::init(argc, argv, "raw_SIFT_feature_matching_node");
    ros::NodeHandle nh;
    

    ros::Subscriber raw_image_sub = nh.subscribe<sensor_msgs::Image>("/raw_image", 1, ImgSubCallback);
    

    if(ros::ok())
        ros::spin();


    return 0;
}
