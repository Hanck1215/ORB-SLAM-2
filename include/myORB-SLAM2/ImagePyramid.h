#ifndef IMAGEPYRAMID_H
#define IMAGEPYRAMID_H

#include <vector>
#include <list>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

class ImagePyramid {
    public :
        /*
        @brief 影像金字塔
        
        @param[in] nLevels 影像金字塔的層數
        @param[in] fScaleFactor 每層之間的縮放因子 (例如 1.2) 
        @param[in] nFeatures 總共需要提取的特徵點數量 */
        ImagePyramid (int nLevels, float fScaleFactor, int nFeatures);
        ~ImagePyramid () {};

        // 設定影像 : 將不同縮放倍率的影像依序放入影像金字塔中
        void setImage(const Mat &image);

        int mnLevels; // 影像金字塔的層數
        int mnFeatures; // 總共需要提取的特徵點數量
        float mScaleFactor; // 每層之間的縮放係數

        vector<int> mvnFeaturesPerLevel; // 儲存每一層影像中應提取的「特徵點數」
        vector<float> mvScaleFactors; // 儲存每一層影像相較於第一層影像的「縮小倍數」
        vector<float> mvInvScaleFactors; // 儲存每一層影像恢復為第一層影像大小所需的「縮放倍數」
        vector<Mat> mvImages; // 儲存每一層影像的矩陣
    
    private :
        // 輸出影像金字塔相關資訊
        void info() {
            printf("Image Pyramid Information: \n");
            printf(" - Levels: %d\n", mnLevels);
            printf(" - Scale Factor: %f\n", mScaleFactor);
            printf(" - Number of Features: %d\n", mnFeatures);
            
            printf("\n - Features Per Level: \n");
            printf(" { ");
            for(int i = 0; i < mnLevels-1; i++) { printf("%d, ", mvnFeaturesPerLevel[i]); }
            printf("%d", mvnFeaturesPerLevel[mnLevels-1]);
            printf(" }\n");

            printf("\n - Scale Factors: \n");
            printf(" { ");
            for(int i = 0; i < mnLevels-1; i++) { printf("%f, ", mvScaleFactors[i]); }
            printf("%f", mvScaleFactors[mnLevels-1]);
            printf(" }\n");

            printf("\n - Inv Scale Factors: \n");
            printf(" { ");
            for(int i = 0; i < mnLevels-1; i++) { printf("%f, ", mvInvScaleFactors[i]); }
            printf("%f", mvInvScaleFactors[mnLevels-1]);
            printf(" }\n");
        }
};

}
#endif