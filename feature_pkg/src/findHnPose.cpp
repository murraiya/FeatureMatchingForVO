#include "wholeprocess.hpp"


int gettingH(){
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    H = findHomography( obj, scene, RANSAC );
   

    return 0;
}

