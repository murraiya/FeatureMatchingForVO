#include "wholeprocess.hpp"


int gettingH(const vector<cv::DMatch>& good_matches){
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        beforeFramePt.push_back( ReferenceKeypoints[ good_matches[i].queryIdx ].pt );
        afterFramePt.push_back( TargetKeypoints[ good_matches[i].trainIdx ].pt );
    }
    H = findHomography( beforeFramePt, afterFramePt, RANSAC );
    cv::recoverPose(E, points2, points1, R, t, 1400 , pp, mask);

    //start here!!!!!!!!!!!!! recover pose input E F H ??

    return 0;
}

