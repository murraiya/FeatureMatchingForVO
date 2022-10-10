#include "feature_pkg/wholeprocess.hpp"


int gettingPose(vector<cv::KeyPoint> keypoints_before, vector<cv::KeyPoint> keypoints_after, vector<cv::DMatch> good_matches){
    cout<<"in here gettingPose"<<endl;
    
    beforept.clear();
    afterpt.clear();
    
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        beforept.push_back( keypoints_before[ good_matches[i].queryIdx ].pt );
        afterpt.push_back( keypoints_after[ good_matches[i].trainIdx ].pt );
    }

    E = findEssentialMat(beforept, afterpt, intrinsic, RANSAC, 0.999, 1.0);
    std::cout<<E<<endl;

    cv::recoverPose(E, beforept, afterpt, intrinsic, R, t);

    ///////////VISUALIZATION/////////////

    // poseMessage.translation=

    // cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
    // T( cv::Range(0,3), cv::Range(0,3) ) = R * 1; // copies R into T
    // T( cv::Range(0,3), cv::Range(3,4) ) = t * 1; // copies t into T

    // T is a 4x4 matrix with the pose of the camera in the object frame

    R_f=R.clone();
    t_f=t.clone();

    scale = abs(t.at<double>(2));

    cout << "Scale is " << scale << endl;

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;

    }

    x = int(t_f.at<double>(0)) + 300;
    y = int(t_f.at<double>(2)) + 100;


    cout<<Point(x,y)<<endl;

    cout<<"fuckyouuuuuuuuuuu"<<endl;


    //here makes process down 
    cv::circle(traj, Point(x, y), 1, Scalar(255,0,0), 2);
    cout<<"PROBLEM? 6666"<<endl;

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), FILLED); //or CV_FILLED
    
    
    // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    // putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    cout<<"PROBLEM? 7777"<<endl;

    imshow( "Trajectory", traj );

    waitKey(1);


    return 0;
}
