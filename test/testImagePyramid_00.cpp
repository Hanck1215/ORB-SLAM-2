#include "myORB-SLAM2/ImagePyramid.h" // 引入自定義的 ImagePyramid 類別
#include <opencv2/opencv.hpp> // 引入 OpenCV 的核心功能和影像處理功能
#include <chrono> // 提供高精度時間測量功能

using namespace my_ORB_SLAM2; // 使用自定義命名空間 my_ORB_SLAM2
using namespace std; // 使用標準命名空間
using namespace cv; // 使用 OpenCV 的命名空間

// test cmd: ./testImagePyramid_00 ../../images_00/000000.png
// 測試命令：執行程式並傳入影像路徑作為參數
int main(int argc, char **argv) {
    string path = argv[1]; // 從命令列參數中獲取影像路徑
    Mat image = imread(path); // 使用 OpenCV 的 imread 函式讀取影像

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
    printf("Setting Images costs: %f\n", time_used.count());

    // 獲取影像金字塔的所有層影像
    vector<Mat> images = imagePyramid.mvImages;

    // 遍歷影像金字塔的每一層，並顯示影像
    int level = 0;
    for(Mat &image : images) { 
        string name = "image"; // 設定視窗名稱的前綴
        name.append(to_string(level++)); // 為每層影像添加層數後綴
        imshow(name, image); // 使用 OpenCV 的 imshow 函式顯示影像
    }

    // 等待使用者按下任意鍵後關閉所有視窗
    waitKey(0);
    destroyAllWindows();
    return 0; // 程式執行成功
}