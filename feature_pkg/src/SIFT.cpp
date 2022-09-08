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

    imshow("SIFT Video", frame);
    int key = cv::waitKey(1);
    // cout<<"key : "<<key<<endl;
    cout<<"frame: "<< cnt++ <<endl;

    if(cnt)
    {
        cout << "References image으로 선택됩니다.\n\n";
        Refer_image = frame; // 눌렀을 때의 해당 프레임을 Reference frame으로 선택
        cv::cvtColor(Refer_image, Refer_gray_image, cv::COLOR_RGB2GRAY);

        // imshow("Reference Image", Refer_image);			// References image 보여주기
        // imshow("Reference Gray Image", Refer_gray_image);
    }

    if(++cnt)
    // SIFT를 이용, Target image 선정
    {
        sift->detectAndCompute(Refer_gray_image, cv::Mat(), ReferenceKeypoints, ReferDescriptor);

        // Refer image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
        /*cv::Mat sift_refer_result;
        cv::drawKeypoints(Refer_gray_image, ReferenceKeypoints, sift_refer_result);
        imshow("Result_sift_)resfer_result", sift_refer_result);*/

        cout << "SIFT를 선택하셨습니다. 해당 이미지를 Target image으로 선택합니다.\n\n";
        Target_image = frame; // 눌렀을 때의 해당 프레임을 Target image으로 선택
        cv::cvtColor(Target_image, Target_gray_image, cv::COLOR_RGB2GRAY);

        //imshow("Target Image", Target_image);	// Target image 보여주기
        // imshow("Target Gray Image", Target_gray_image);

        //sift->detect(Target_gray_image, TargetKeypoints);
        //sift->compute(Target_gray_image, TargetKeypoints, TargetDescriptor);   // 예전 버전의 경우 이렇게 했어야 함
        sift->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);

        // Target image에 대해 keypoint 찾은 거 보고 싶으면 주석 해제
        /*cv::Mat sift_target_result;
        cv::drawKeypoints(Target_gray_image, TargetKeypoints, sift_target_result);
        imshow("Result_sift_Target_result", sift_target_result);*/

        // Refer image와 Target image의 fature를 이용하여 Feature matching 실행

        Matcher_SIFT->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.
        Matcher_SIFT->match(ReferDescriptor, TargetDescriptor, matches);

        double max_dist = 0.0, min_dist = 100.0;
        for (int i = 0; i < matches.size(); i++)
        {
            double dist = matches[i].distance;
            if (dist < min_dist)
                min_dist = dist;
            if (dist > max_dist)
                max_dist = dist;
        }

        // drawing only good matches (dist less than 2*min_dist)
        vector<cv::DMatch> good_matches;

        for (int i = 0; i < matches.size(); i++)
        {
            if (matches[i].distance <= 2 * min_dist)
            {
                good_matches.push_back(matches[i]);
            }
        }

        cv::Mat Result_SIFT;

        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, good_matches, Result_SIFT, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.

        imshow("Result_SIFT", Result_SIFT);
        // imwrite("/home/autonav/feature_ws/Matching_SIFT.jpg", Result_SIFT);

        cv::waitKey(1);

    }
}

int main(int argc, char** argv) {
    cout<<"! SIFT !"<<endl;

    ros::init(argc, argv, "SIFT_feature_matching_node");
    ros::NodeHandle nh;
    
    ros::Subscriber raw_image_sub = nh.subscribe<sensor_msgs::Image>("/raw_image", 1, ImgSubCallback);
    
    if(ros::ok())
        ros::spin();


    return 0;
}
