#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int ac, char** av) {

	VideoCapture cap("/media/autonav/SJ_SSD/forFeatureMatching.avi");
	if (!cap.isOpened())
	{
		printf("Can't open the camera");
		return -1;
	}

	Mat img;

	while (1)
	{
		cap >> img;
        
		if (img.empty())
		{
			printf("empty image");
			return 0;
		}
        
		imshow("camera img", img);

		if (waitKey(25) == 27)
			break;
	}


	return 0;
}