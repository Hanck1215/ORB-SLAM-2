#ifndef IMAGEPYRAMID_H
#define IMAGEPYRAMID_H

#include <vector>
#include <list>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

class ImagePyramid {
    public :
        /*
        @brief 設定影像金字塔參數
        
        @param[in] m 影像金字塔的層數
        @param[in] s 每層之間的縮放係數 (例如 1.2) */
        ImagePyramid (int m, int s);
        ~ImagePyramid () {};

    private :
        int mnLevels; // 影像金字塔的層數
        double mScaleFactor; // 每層之間的縮放係數

    private :
        vector<int> mvnFeaturesPerLevel; // 儲存每一層影像中應提取的特徵點數
        vector<float> mvScaleFactor; // 儲存每一層影像相較於第一層影像的縮小倍數


};

}
#endif