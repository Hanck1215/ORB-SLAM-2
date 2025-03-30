#include "include/myORB-SLAM2/ImagePyramid.h"
#include <opencv2/imgproc.hpp>

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
        for(int level = 1; level < mnLevels; level++) {
            mvScaleFactor[level] = mvScaleFactor[level-1] * mScaleFactor;
            mvInvScaleFactor[level] = 1.0f / mvScaleFactor[level];
        }

        // 計算影像金字塔中每一層應提取的特徵點數量 (根據公式)
        int sumFeatures = 0;
        float factor = 1.0f / mScaleFactor;
        float nDesiredFeaturesPerScale = (mnPoints*(1-factor)) / (1-(float)pow((double)factor, (double)mnLevels));
        for(int level = 1; level < mnLevels-1; level++) {
            mvnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mvnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor; 
        }
        mvnFeaturesPerLevel[mnLevels-1] = std::max(mnPoints-sumFeatures, 0);
    }

    /*
    @brief 設定影像 : 將不同縮放倍率的影像依序放入影像金字塔中
    
    @param[in] image 影像金字塔的影像*/
    void ImagePyramid::setImage(Mat image) {
        image.copyTo(mvImage[0]);
        for (int level = 1; level < mnLevels; ++level) {
            float scale = mvInvScaleFactor[level];
            Size size(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
            resize(mvImage[level-1], mvImage[level], size, 0, 0, INTER_LINEAR);
        }
    }
}