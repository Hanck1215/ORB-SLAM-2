#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractorPyramid.h"

#include <opencv2/opencv.hpp>
#include <chrono>

using namespace my_ORB_SLAM2;

int main(int argc, char **argv) {
    // 讀取指定影像並設定影像金字塔
    string path = argv[1];
    Mat image = imread(path, 0);
    ImagePyramid imagePyramid(3, 1.2, 2000);

    // 宣告關鍵點提取器，以及關鍵點金字塔容器
    vector<vector<KeyPoint>> vvKeyPointsPerLevel;
    KeyPointExtractorPyramid KeyPointExtractorPyramid(imagePyramid.mnLevels, 30.0f, 19, imagePyramid.mnPoints, 20, 7);
    
    // 設定影像到影像金字塔
    imagePyramid.setImage(image);

    // 提取關鍵點
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    KeyPointExtractorPyramid.extract(vvKeyPointsPerLevel, imagePyramid.mvImages);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = 
    std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("\nExtracting key points costs: %f\n", time_used.count());

    // 顯示影像金字塔中，每一層的影像
    int level = 0;
    cv::Mat outImage;
    for(Mat &image : imagePyramid.mvImages) {
        cv::drawKeypoints(image, vvKeyPointsPerLevel[level++], outImage, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        string windowName = "KeyPoints";
        windowName.append(to_string(level));
        cv::imshow(windowName, outImage);
    }
    
    char key = '\0';
    while(key != 'q') {
        key = cv::waitKey(0);
    }
    
    cv::destroyAllWindows();
}