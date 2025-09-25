#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractorPyramid.h"
#include "myORB-SLAM2/KeyPointDistributorPyramid.h"

#include <opencv2/opencv.hpp>
#include <chrono>

#include <dirent.h>

using namespace my_ORB_SLAM2;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    // 指定影像序列資料夾路徑
    string dirPath = argv[1];

    // 儲存每一個影像檔案名稱
    std::vector<std::string> vImageFileNames;
    vImageFileNames.reserve(4071);
    for (int i = 0; i < 4071; i++) {
        std::ostringstream ss;
        ss << std::setw(6) << std::setfill('0') << i << ".png";
        vImageFileNames.push_back(ss.str());
    }

    // 設定影像金字塔、關鍵點提取器、關鍵點均勻器
    ImagePyramid imagePyramid(3, 1.2, 2000);
    vector<vector<KeyPoint>> vvKeyPointsPerLevel;
    KeyPointExtractorPyramid keyPointExtractorPyramid(imagePyramid.mnLevels, 30.0f, 19, imagePyramid.mnPoints, 20, 7);
    KeyPointDistributorPyramid keyPointDistributorPyramid;
    
    for(string imageFileName : vImageFileNames) {
        Mat image = imread(dirPath + "/" + imageFileName);
        // 計算關鍵點提取&均勻化所花費的時間
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // 關鍵點提取&均勻化
        imagePyramid.setImage(image);
        vector<Mat> &vImagePerLevel = imagePyramid.mvImages;
        keyPointExtractorPyramid.extract(vvKeyPointsPerLevel, vImagePerLevel);
        vector<vector<KeyPoint>> vvDistributedKeyPointsPerLevel;
        keyPointDistributorPyramid.distribute(
            vvDistributedKeyPointsPerLevel,
            vvKeyPointsPerLevel,
            imagePyramid.mvnFeaturesPerLevel,
            vImagePerLevel
        );
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = 
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        printf("\nKey points extraction and distribution costs: %f\n", time_used.count());

        cv::Mat outImage;
        cv::drawKeypoints(image, vvDistributedKeyPointsPerLevel[0], outImage, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        string windowName = "Distributed Key Points";
        cv::imshow(windowName, outImage);

        if(cv::waitKey(0) == 'q') { break; }
    }
    
    cv::destroyAllWindows();
}