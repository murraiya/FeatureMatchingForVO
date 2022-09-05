#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace cv;
using namespace std;

cv_bridge::CvImage cvImg;
sensor_msgs::Image ros_img_msg;
std_msgs::Header header;
Mat img;


void ROSpub(Mat& img){
    cout<<img.empty()<<endl;
    ros::NodeHandle nh;
    ros::Publisher raw_image_pub = nh.advertise<sensor_msgs::ImagePtr>("/raw_image", 1);
    cvImg.image=img;
    cvImg.toImageMsg(ros_img_msg);
    
    raw_image_pub.publish(ros_img_msg);
    cout<<"sensormsg pub"<<endl;
};


// cv::Mat img; // << image MUST be contained here
// cv_bridge::CvImage img_bridge;
// sensor_msgs::Image img_msg; // >> message to be sent

// std_msgs::Header header; // empty header
// header.seq = counter; // user defined counter
// header.stamp = ros::Time::now(); // time
// img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
// img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
// pub_img.publish(img_msg); // ros::Publi

int loadVideo(){

	VideoCapture cap("/media/autonav/SJ_SSD/forFeatureMatching.avi");
	if (!cap.isOpened())
	{
		printf("Can't open the camera");
        return -1;
	}

	

	while (1)
	{
		cap >> img;
        
		if (img.empty())
		{
			printf("empty image");
            return 0;
		}

        ROSpub(img);

        
		//imshow("camera img", img);

		// if (waitKey(25) == 27)
		// 	break;
	}


}



int main(int argc, char** argv) {

    ros::init(argc, argv, "videoLoadNode");
    
    loadVideo();
    return 0;
}