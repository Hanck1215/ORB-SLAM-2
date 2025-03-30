#include "myORB-SLAM2/ImagePyramid.h"
#include <opencv2/opencv.hpp>

using namespace my_ORB_SLAM2;
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    string path = argv[1];
    Mat image = imread(path);

    ImagePyramid imagePyramid(8, 1.2, 2000);
    imagePyramid.setImage(image);

    vector<Mat> images = imagePyramid.getImages();
    for(Mat &image : images) { imshow("image", image); }
    waitKey(0);
    destroyAllWindows();
    return 0;
}