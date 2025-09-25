#ifndef KEYPOINTDISTRIBUTOR
#define KEYPOINTDISTRIBUTOR

#include "myORB-SLAM2/KeyPointsRegionalQuadTree.h"

namespace my_ORB_SLAM2 {

class KeyPointDistributorPyramid {
    public:
        KeyPointDistributorPyramid() {};
        ~KeyPointDistributorPyramid() {};

        void distribute(
            const vector<int> &nFeaturesPerLevel,
            vector<vector<KeyPoint>> &resultKeyPointsPerLevel,
            vector<vector<KeyPoint>> &keyPointsPerLevel, 
            const vector<Mat> &imagesPerLevel
        );
};

}

#endif