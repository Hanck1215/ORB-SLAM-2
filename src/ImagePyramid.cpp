#include "myORB-SLAM2/ImagePyramid.h"

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
        mvScaleFactors.resize(mnLevels);
        mvInvScaleFactors.resize(mnLevels);
        mvImages.resize(mnLevels);

        // 初始化影像金字塔中，每一層的縮小倍數與反向縮放倍數
        mvScaleFactors[0] = 1.0f;
        mvInvScaleFactors[0] = 1.0f;
        for(int level = 1; level < mnLevels; level++) {
            mvScaleFactors[level] = mvScaleFactors[level-1] * mScaleFactor;
            mvInvScaleFactors[level] = 1.0f / mvScaleFactors[level];
        }

        // 計算影像金字塔中每一層應提取的特徵點數量 (根據公式)
        int sumFeatures = 0;
        float factor = 1.0f / mScaleFactor;
        float nDesiredFeaturesPerScale = (mnPoints*(1-factor)) / (1-(float)pow((double)factor, (double)mnLevels));
        for(int level = 0; level < mnLevels-1; level++) {
            mvnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mvnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor; 
        }
        mvnFeaturesPerLevel[mnLevels-1] = max(mnPoints-sumFeatures, 0);

        info(); // 輸出影像金字塔相關資訊
    }

    /*
    @brief 設定影像 : 將不同縮放倍率的影像依序放入影像金字塔中
    
    @param[in] image 影像金字塔的影像*/
    void ImagePyramid::setImage(const Mat &image) {
        mvImages[0] = image;
        for (int level = 1; level < mnLevels; ++level) {
            float scale = mvInvScaleFactors[level];
            Size size(cvRound(image.cols*scale), cvRound(image.rows*scale));
            resize(mvImages[level-1], mvImages[level], size, 0, 0, INTER_NEAREST);
        }
    }
}