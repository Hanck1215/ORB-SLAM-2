#include "myORB-SLAM2/ImagePyramid.h"
#include "myORB-SLAM2/KeyPointExtractor.h"

#include <opencv2/opencv.hpp>
#include <chrono>

using namespace my_ORB_SLAM2;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    string path = argv[1];
    Mat image = imread(path, 0);

    // 建立影像金字塔物件，設定層數為 3，每層縮放係數為 1.2，待提取的總特徵點數量為 2000
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ImagePyramid imagePyramid(3, 1.2, 2000);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = 
    std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("\nSetting Parameters costs: %f\n", time_used.count());

    // 將讀取的影像設置為影像金字塔的第一層，並生成其他層
    t1 = std::chrono::steady_clock::now();
    imagePyramid.setImage(image);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("Setting Images costs: %f\n\n", time_used.count());

    KeyPointExtractor keyPointExtractor(imagePyramid.getLevels(), 30.0f, 19, imagePyramid.getPoints(), 20, 7, 31);
    vector<vector<KeyPoint>> keyPointsPerLavel;
    const vector<Mat> &images = imagePyramid.getImages();
    const vector<float> &scaleFactors = imagePyramid.getScaleFactors();

    t1 = std::chrono::steady_clock::now();
    keyPointExtractor.extract(keyPointsPerLavel, images, scaleFactors);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    printf("Extracting keypoints costs: %f\n\n", time_used.count());

    for(int i = 0; i < images.size(); i++) {
        cv::Mat outImg;
        cv::drawKeypoints(images[i], keyPointsPerLavel[i], outImg, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);

        cv::imshow("Keypoints", outImg);
        cv::waitKey(0);
    }

}