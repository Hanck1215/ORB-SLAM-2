#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {
    class ORBExtractor {
        public :
            ORBExtractor (int nFeatures, float scaleFactor, int nLevels);
            ~ORBExtractor () {};
        
        private : 
            int mnLevels;
            int mnFeatures;
            float mScaleFactor;

            vector<int> mvnFeaturesPerLevel;

            vector<float> mvScaleFactor;
            vector<float> mvLevelSigma2;
            vector<float> mvInvScaleFactor;
            vector<float> mvInvLevelSigma2;

            vector<Mat> mvImagePyramid;
        
        private : 
            void configurePyramidInfo();
            void allocateORBToPyramid();
    };
}

#endif