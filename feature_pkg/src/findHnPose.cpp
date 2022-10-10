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

    // double l = 5;
    // std::vector<cv::Point2f> pose_points2d;
    // pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,0)));    // axis center
    // pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(l,0,0)));    // axis x
    // pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,l,0)));    // axis y
    // pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,l)));    // axis z
    // draw3DCoordinateAxes(frame_vis, pose_points2d);     

    return 0;
}

                                  // draw axes

/*
def draw_axis(img, R, t, K):
    
    rotV, _ = cv2.Rodrigues(R)
    points = np.float32([[100, 0, 0], [0, 100, 0], [0, 0, 100], [0, 0, 0]]).reshape(-1, 3)
    axisPoints, _ = cv2.projectPoints(points, rotV, t, K, (0, 0, 0, 0))
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
    return img
*/