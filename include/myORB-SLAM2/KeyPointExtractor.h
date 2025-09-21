#ifndef KEYPOINTEXTRACTOR_H
#define KEYPOINTEXTRACTOR_H

#include <vector>

#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

class KeyPointExtractor {
    public:
        /*
        @brief 設定關鍵點提取器
        
        @param[in] nLevels 影像金字塔的層數
        @param[in] defaultGridSize 預設每個小格子的尺寸
        @param[in] paddingPixels 邊緣向內部填充的 pixels 數量
        @param[in] nKeyPoints 總共要提取的關鍵點數量
        @param[in] maxTh 一開始提取關鍵點時使用的閾值
        @param[in] minTh 放寬標準後，提取關鍵點的閾值 */
        KeyPointExtractor(int nLevels, float defaultGridSize, int paddingPixels, int nKeyPoints, int maxTh, int minTh);
        ~KeyPointExtractor() {};

        /*
        @brief 提取關鍵點的 function
        
        @param[in, out] keyPointsPerLevel 儲存影像金字塔中，每層影像的關鍵點
        @param[in] imagesPerLevel 影像金字塔的每一層影像
        @param[in] scaleFactors 每一層影像轉換到第一層尺度的縮放因子 */
        void extract(
            vector<vector<KeyPoint>> &keyPointsPerLevel, 
            const vector<Mat> &imagesPerLevel,
            const vector<float> &scaleFactors
        );
    
    private:
        int nLevels, paddingPixels, nKeyPoints, maxTh, minTh;
        float defaultGridSize;
};

}
#endif