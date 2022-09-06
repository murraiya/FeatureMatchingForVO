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
// #include <opencv2/xfeatures2d.hpp>


using namespace std;


cv::Mat frame;
cv::Mat Refer_image;
cv::Mat Refer_gray_image;
cv::Mat Target_image;
cv::Mat Target_gray_image;

vector<cv::KeyPoint> TargetKeypoints, ReferenceKeypoints;
cv::Mat TargetDescriptor, ReferDescriptor;

cv::Ptr<cv::Feature2D> orb = cv::ORB::create(10);
// cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(500);
cv::Ptr<cv::Feature2D> brisk = cv::BRISK::create();

cv::Ptr<cv::DescriptorMatcher> Matcher_ORB = cv::BFMatcher::create(cv::NORM_HAMMING);		// Brute-Force matcher create method
cv::Ptr<cv::DescriptorMatcher> Matcher_BRISK = cv::BFMatcher::create(cv::NORM_HAMMING);		// Brute-Force matcher create method
// cv::Ptr<cv::DescriptorMatcher> Matcher_SIFT = cv::BFMatcher::create(cv::NORM_L2);			// Brute-Force matcher create method

vector<cv::DMatch> matches;	// Class for matching keypoint descriptors.

void processing(cv::Mat&);


void ImgSubCallback(const sensor_msgs::Image raw_img){
    // cout<<"img_sub"<<endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_img, sensor_msgs::image_encodings::BGR8);
    //cout<<cv_ptr->image.type()<<endl;
    processing(cv_ptr->image);
    //cout<<cv_ptr->image<<endl;
}


void processing(cv::Mat& frame){
     // 종료 시까지 영상 출력
    
    if(frame.empty())
        cout<<"NO IMG NO IMG NO IMG"<<endl;
    imshow("Video", frame);		// 현재 웹캠을 통해 실시간으로 받고 있는 영상을 보여주기

    int key = cv::waitKey(1);


    if ((key == 82) || (key == 114)) // 녹화 버튼(R 또는 r)을 눌렀을 때
    {
        cout << "References image으로 선택됩니다.\n\n";
        Refer_image = frame; // 눌렀을 때의 해당 프레임을 Reference frame으로 선택
        cv::cvtColor(Refer_image, Refer_gray_image, cv::COLOR_RGB2GRAY);

        imshow("Reference Image", Refer_image);			// References image 보여주기
        imshow("Reference Gray Image", Refer_gray_image);
    }



    // 이제 타겟 이미지를 정해서 Refer frame과 feature matching을 실행할 것임

    // O or o : ORB , B or b : BRISK, S or s : SIFT를 선택할 수 있도록 설정하기

    //if ((key == 84) || (key == 116)) // Target Image(T 또는 t)을 눌렀을 때
        if ((key == 82) || (key == 114)) // 녹화 버튼(R 또는 r)을 눌렀을 때
    {
        cout << "References image으로 선택됩니다.\n\n";
        Refer_image = frame; // 눌렀을 때의 해당 프레임을 Reference frame으로 선택
        cv::cvtColor(Refer_image, Refer_gray_image, cv::COLOR_RGB2GRAY);

        imshow("Reference Image", Refer_image);			// References image 보여주기
        imshow("Reference Gray Image", Refer_gray_image);
    }
    if((key == 79) || (key == 111))	// ORB를 이용, Target image 선정
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

        imshow("Target Image", Target_image);	// Target image 보여주기
        imshow("Target Gray Image", Target_gray_image);

        // Target image에 대해 feature 뽑기
        orb->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);// detects keypoints and computes the descriptors

        // Refer image와 Target image의 fature를 이용하여 Feature matching 실행
        Matcher_ORB->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.

        sort(matches.begin(), matches.end());
        const int match_size = matches.size();
        vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.5f));

        cv::Mat Result;
        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, matches, Result, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.

        imshow("Matching Result_ORB", Result);
    }
    else if ((key == 66) || (key == 98))	// BRISK를 이용, Target image 선정
    {
        brisk->detectAndCompute(Refer_gray_image, cv::Mat(), ReferenceKeypoints, ReferDescriptor);

        cout << "BRISK를 선택하셨습니다. 해당 이미지를 Target image으로 선택합니다.\n\n";
        Target_image = frame; // 눌렀을 때의 해당 프레임을 Target image으로 선택
        cv::cvtColor(Target_image, Target_gray_image, cv::COLOR_RGB2GRAY);
        imshow("Target Image", Target_image);	// Target image 보여주기
        imshow("Target Gray Image", Target_gray_image);

        brisk->detectAndCompute(Target_gray_image, cv::Mat(), TargetKeypoints, TargetDescriptor);

        Matcher_BRISK->match(TargetDescriptor, ReferDescriptor, matches);	// Find the best match for each descriptor from a query set.
        Matcher_BRISK->match(ReferDescriptor, TargetDescriptor, matches);

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

        cv::Mat Result_BRISK;

        cv::drawMatches(Target_gray_image, TargetKeypoints, Refer_gray_image, ReferenceKeypoints, good_matches, Result_BRISK, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // Draws the found matches of keypoints from two images.

        imshow("Matching Result_BRISK", Result_BRISK);
    }
}

int main(int argc, char** argv) {


    cout << "R 또는 r 버튼을 누르시면 References frame으로 선택됩니다. \n";
    cout << "Reference Image를 선택한 후에 Target Image를 선택하시오. \n";
	cout << "그 후 o(ORB), b(BRISK), s(SIFT)를 선택하시오.\n";



    ros::init(argc, argv, "videoSubVisualNode");
    ros::NodeHandle nh;
    
    ros::Subscriber raw_image_sub = nh.subscribe<sensor_msgs::Image>("/raw_image", 1, ImgSubCallback);
    
    if(ros::ok())
        ros::spin();


    return 0;
}
