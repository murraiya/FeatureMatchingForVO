#include "wholeprocess.hpp"


int gettingH(const vector<cv::DMatch>& good_matches){
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        beforeFramePt.push_back( ReferenceKeypoints[ good_matches[i].queryIdx ].pt );
        afterFramePt.push_back( TargetKeypoints[ good_matches[i].trainIdx ].pt );
    }
    H = findHomography( beforeFramePt, afterFramePt, RANSAC );
   

    return 0;
}

