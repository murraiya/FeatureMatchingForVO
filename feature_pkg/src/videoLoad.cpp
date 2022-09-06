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

Mat img;

sensor_msgs::Image loadVideo(){

	VideoCapture cap("/media/autonav/SJ_SSD/forFeatureMatching.avi");
	if (!cap.isOpened())
	{
		printf("Can't open the camera");
	}

	while(1){


		cap >> img;
		
		if (img.empty())
		{
			printf("empty image");
		}

		 cout<<"after_pub"<<endl;

		sensor_msgs::Image ros_img_msg;

		cout<<img.empty()<<endl;
		
		std_msgs::Header header; // empty header
		header.seq = 1; // user defined counter
		header.stamp = ros::Time::now(); // time

		cv_bridge::CvImage cvImg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
		cvImg.toImageMsg(ros_img_msg);
        
		return ros_img_msg;
		
    
	}

}



int main(int argc, char** argv) {

    ros::init(argc, argv, "videoLoadNode");
    ros::NodeHandle nh;
	ros::Publisher raw_image_pub = nh.advertise<sensor_msgs::Image>("/raw_image", 10);

    while(ros::ok()){
		raw_image_pub.publish(loadVideo());
	}
    return 0;
}