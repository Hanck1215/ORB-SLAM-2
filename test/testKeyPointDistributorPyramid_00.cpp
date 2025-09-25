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
    Mat RGBImage = imread(path);

    // 宣告影像金字塔
    ImagePyramid imagePyramid(3, 1.2, 2000);

    // 宣告關鍵點提取器
    vector<vector<KeyPoint>> keyPointsPerLavel;
    KeyPointExtractorPyramid keyPointExtractorPyramid(imagePyramid.getLevels(), 30.0f, 19, imagePyramid.getPoints(), 20, 7);
    KeyPointDistributorPyramid keyPointDistributorPyramid;
    
    // 設定影像到影像金字塔
    imagePyramid.setImage(image);
    const vector<Mat> &images = imagePyramid.getImages();

    // 提取關鍵點
    keyPointExtractorPyramid.extract(keyPointsPerLavel, images);

    // 關鍵點均勻化
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    vector<vector<KeyPoint>> resultKeyPointsPerLevel;
    keyPointDistributorPyramid.distribute(imagePyramid.getFeaturesPerLevel(), resultKeyPointsPerLevel, keyPointsPerLavel, images);
     
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = 
    std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("\nkeyPointDistributorPyramid costs: %f\n", time_used.count());

    for (const auto& kv : resultKeyPointsPerLevel) {
        printf("%d keypoints\n", (int)kv.size());
    }

    // 比較兩者差異
    cv::Mat outImg;
    
    cv::drawKeypoints(RGBImage, resultKeyPointsPerLevel[0], outImg, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("KeyPoints", outImg);

    char key = '\0';
    while(key != 'q') {
        key = cv::waitKey(0);
    }
    
    cv::destroyAllWindows();
}