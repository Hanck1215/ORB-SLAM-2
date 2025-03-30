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
        @param[in] s 每層之間的縮放係數 (例如 1.2) 
        @param[in] N 總共需要提取的特徵點數量 */
        ImagePyramid (int m, int s, int N);
        ~ImagePyramid () {};

        // 存取影像金字塔的層數
        int inline getLevels() { return mnLevels; }

    private :
        int mnLevels; // 影像金字塔的層數
        int mnPoints; // 總共需要提取的特徵點數量
        double mScaleFactor; // 每層之間的縮放係數

    private :
        vector<int> mvnFeaturesPerLevel; // 儲存每一層影像中應提取的「特徵點數」
        vector<float> mvScaleFactor; // 儲存每一層影像相較於第一層影像的「縮小倍數」
        vector<float> mvInvScaleFactor; // 儲存每一層影像恢復為第一層影像大小所需的「縮放倍數」


};

}
#endif