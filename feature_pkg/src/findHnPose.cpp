#include "feature_pkg/wholeprocess.hpp"


int gettingPose(vector<cv::KeyPoint> keypoints_before, vector<cv::KeyPoint> keypoints_after, vector<cv::DMatch> good_matches){
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        beforept.push_back( keypoints_before[ good_matches[i].queryIdx ].pt );
        afterpt.push_back( keypoints_after[ good_matches[i].trainIdx ].pt );
    }

    E = findEssentialMat(beforept, afterpt, intrinsic, RANSAC, 0.999, 1.0);
    std::cout<<E<<endl;

    cv::recoverPose(E, beforept, afterpt, intrinsic, R, t);

    return 0;
}

