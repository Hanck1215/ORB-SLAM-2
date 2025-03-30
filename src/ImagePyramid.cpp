#include "include/myORB-SLAM2/ImagePyramid.h"

namespace my_ORB_SLAM2 {
    /*
    @brief 設定影像金字塔參數
    
    @param[in] m 影像金字塔的層數
    @param[in] s 每層之間的縮放係數 (例如 1.2) 
    @param[in] N 總共需要提取的特徵點數量 */
    ImagePyramid::ImagePyramid(int m, float s, int N) {
        // 初始化影像金字塔基礎參數
        mnLevels = m;
        mnPoints = N;
        mScaleFactor = s;

        // 初始化資料結構大小以符合影像金字塔層數
        mvnFeaturesPerLevel.resize(mnLevels);
        mvScaleFactor.resize(mnLevels);
        mvInvScaleFactor.resize(mnLevels);
        mvImage.resize(mnLevels);

        // 初始化影像金字塔中，每一層的縮小倍數與反向縮放倍數
        mvScaleFactor[0] = 1.0f;
        mvInvScaleFactor[0] = 1.0f;
        for(int i = 1; i < mnLevels; i++) {
            mvScaleFactor[i] = mvScaleFactor[i-1] * mScaleFactor;
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        }


    }
}