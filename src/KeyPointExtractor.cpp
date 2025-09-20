#include "myORB-SLAM2/KeyPointExtractor.h"

namespace my_ORB_SLAM2 {
    /*
    @brief 設定關鍵點提取器
    
    @param[in] nLevels 影像金字塔的層數
    @param[in] defaultGridSize 預設每個小格子的尺寸
    @param[in] paddingPixels 邊緣向內部填充的 pixels 數量
    @param[in] nKeyPoints 總共要提取的關鍵點數量
    @param[in] maxTh 一開始提取關鍵點時使用的閾值
    @param[in] minTh 放寬標準後，提取關鍵點的閾值
    */
    KeyPointExtractor::KeyPointExtractor(int nLevels, float defaultGridSize, int paddingPixels, int nKeyPoints, int maxTh, int minTh, int patchSize) {
        this->nLevels = nLevels;
        this->defaultGridSize = defaultGridSize;
        this->paddingPixels = paddingPixels;
        this->nKeyPoints = nKeyPoints;
        this->maxTh = maxTh;
        this->minTh = minTh;
        this->patchSize = patchSize;
    };

    /*
    @brief 提取關鍵點的 function
    
    @param[in, out] keyPointsPerLavel 儲存影像金字塔中，每層影像的關鍵點
    @param[in] imagesPerLevel 影像金字塔的每一層影像
    @param[in] scaleFactors 每一層影像轉換到第一層尺度的縮放因子 */
    void KeyPointExtractor::extract(
        vector<vector<KeyPoint>> &keyPointsPerLavel, 
        const vector<Mat> &imagesPerLevel,
        const vector<float> &scaleFactors
    ) {
        // 設定 keyPointsPerLavel 大小為影像金字塔層數
        keyPointsPerLavel.resize(this->nLevels);

        // 遍歷每一層影像
        for(size_t level = 0; level < this->nLevels; level++) {
            // 設定可提取關鍵點的邊界
            int minBorderX = paddingPixels - 3;
            int minBorderY = minBorderX;
            int maxBorderX = imagesPerLevel[level].cols - paddingPixels + 3;
            int maxBorderY = imagesPerLevel[level].rows - paddingPixels + 3;
            
            // keyPoints 用於儲存該層影像的關鍵點，並預留比預計提取之關鍵點數量多 10 倍的空間
            // 避免動態擴增空間造成的性能消耗
            vector<KeyPoint> keyPoints;
            keyPoints.reserve(nKeyPoints * 10);
            
            // 計算關鍵點提取範圍的長寬
            float width = float(maxBorderX - minBorderX);
            float height = float(maxBorderY - minBorderY);
            
            // 計算目前的關鍵點提取範圍可以畫分成幾個行、列
            int nCols = int(width / defaultGridSize);
            int nRows = int(height / defaultGridSize);

            // 計算每個小格子的寬、高
            int gridWidth = ceil(width / nCols);
            int gridHeight = ceil(height / nRows);

            // 遍歷每個 grid
            for(size_t row = 0; row < nRows; row++) {
                // 計算 FAST 關鍵點需要半徑為 3 的圓形範圍
                // 所以每個 grid 的邊界需要暫時向外擴增 3 單位
                // 才可以確保沒丟失邊界的關鍵點訊息
                int ymin = minBorderY + row * gridHeight;
                int ymax = ymin + gridHeight + 6;

                // 如果 ymin 超過 maxBorderY - 6，代表沒有足夠的半徑 3 可以計算關鍵點，跳過
                if(ymin > maxBorderY - 6) { continue; }
                
                // 限制 ymax 不要超過關鍵點的提取範圍
                if(ymax > maxBorderY) { ymax = maxBorderY; }
                
                // 前置步驟和上面一樣
                for(size_t col = 0; col < nCols; col++) {
                    int xmin = minBorderX + col * gridWidth;
                    int xmax = xmin + gridWidth + 6;
                    if(xmin > maxBorderX - 6) { continue; }
                    if(xmax > maxBorderX) { xmax = maxBorderX; }

                    // 提取該 grid 的關鍵點
                    vector<KeyPoint> gridKeyPoints;
                    gridKeyPoints.reserve(32);
                    FAST(
                        imagesPerLevel[level].rowRange(ymin, ymax).colRange(xmin, xmax), 
                        gridKeyPoints, 
                        maxTh
                    );

                    // 如果沒有檢測到關鍵點，就降低閾值在提取一次
                    if(gridKeyPoints.empty()) {
                        FAST(
                            imagesPerLevel[level].rowRange(ymin, ymax).colRange(xmin, xmax), 
                            gridKeyPoints, 
                            minTh
                        );
                    }

                    // 因為目前提取到的關鍵點做標是相對於該 grid 的，所以需要恢復到相對於整張影像的座標
                    if(!gridKeyPoints.empty()) {
                        for(vector<KeyPoint>::iterator vit = gridKeyPoints.begin(); vit != gridKeyPoints.end(); vit++) {
                            (*vit).pt.x += col * gridWidth;
                            (*vit).pt.y += row * gridHeight;
                            keyPoints.push_back(*vit);
                        }
                    }
                }
            }
            // 計算該層影像適用的 Patch Size 來計算描述子
            // 越高層級的影像相對來說需要越大的 Patch Size
            int scaledPatchSize = patchSize * scaleFactors[level];

            // 修飾該層最後提取到的關鍵點
            for(vector<KeyPoint>::iterator vit = keyPoints.begin(); vit != keyPoints.end(); vit++) {
                (*vit).pt.x += minBorderX; // 增加 offset
                (*vit).pt.y += minBorderY; // 增加 offset
                (*vit).octave = level; // 設定關鍵點所屬的金字塔層級
                (*vit).size = scaledPatchSize; // 設定關鍵點適合的 Patch Size 大小
            }
            
            // 儲存該層的 KeyPoints
            keyPointsPerLavel[level] = keyPoints;
        }
    }
}