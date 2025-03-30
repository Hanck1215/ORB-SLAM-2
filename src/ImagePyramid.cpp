#include "include/myORB-SLAM2/ImagePyramid.h"

namespace my_ORB_SLAM2 {
    /*
    @brief 設定影像金字塔參數
    
    @param[in] m 影像金字塔的層數
    @param[in] s 每層之間的縮放係數 (例如 1.2) 
    @param[in] N 總共需要提取的特徵點數量 */
    ImagePyramid::ImagePyramid(int m, double s, int N) {
        mnLevels = m;
        mnPoints = N;
        mScaleFactor = s;
    }
}