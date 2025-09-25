#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractorPyramid.h"
#include "myORB-SLAM2/KeyPointDistributorPyramid.h"

#include <opencv2/opencv.hpp>
#include <chrono>

using namespace my_ORB_SLAM2;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    // 讀取指定影像
    string path = argv[1];
    Mat image = imread(path, 0);

    // 設定影像金字塔、關鍵點提取器、關鍵點均勻器
    ImagePyramid imagePyramid(3, 1.2, 2000);
    vector<vector<KeyPoint>> vvKeyPointsPerLevel;
    KeyPointExtractorPyramid keyPointExtractorPyramid(imagePyramid.mnLevels, 30.0f, 19, imagePyramid.mnPoints, 20, 7);
    KeyPointDistributorPyramid keyPointDistributorPyramid;
    
    // 設定影像到影像金字塔
    imagePyramid.setImage(image);
    vector<Mat> &vImagePerLevel = imagePyramid.mvImages;

    // 提取關鍵點
    keyPointExtractorPyramid.extract(vvKeyPointsPerLevel, vImagePerLevel);

    // 計算關鍵點均勻化所花費的時間
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 關鍵點均勻化
    vector<vector<KeyPoint>> vvDistributedKeyPointsPerLevel;
    keyPointDistributorPyramid.distribute(
        vvDistributedKeyPointsPerLevel,
        vvKeyPointsPerLevel,
        imagePyramid.mvnFeaturesPerLevel,
        vImagePerLevel
    );

    printf("\n\nDistributed Key Points Information:\n");
    for(int iLevel = 0; iLevel < vvDistributedKeyPointsPerLevel.size(); ++iLevel) {
        printf(" - Level %d, key points size %d\n", iLevel, (int)vvDistributedKeyPointsPerLevel[iLevel].size());
    }
     
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = 
    std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("\nKey points distribution costs: %f\n", time_used.count());

    // 顯示影像金字塔中，每一層的影像
    int level = 0;
    for(Mat &image : imagePyramid.mvImages) {
        cv::Mat outImage;
        cv::drawKeypoints(image, vvKeyPointsPerLevel[level++], outImage, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        string windowName = "All Key Points";
        windowName.append(to_string(level));
        cv::imshow(windowName, outImage);
    }

    level = 0;
    for(Mat &image : imagePyramid.mvImages) {
        cv::Mat outImage;
        cv::drawKeypoints(image, vvDistributedKeyPointsPerLevel[level++], outImage, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        string windowName = "Distributed Key Points";
        windowName.append(to_string(level));
        cv::imshow(windowName, outImage);
    }

    char key = '\0';
    while(key != 'q') {
        key = cv::waitKey(0);
    }
    
    cv::destroyAllWindows();
}