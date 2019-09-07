#include "vision/detector.h"
#include "vision/vision.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
using namespace std;
using namespace hirop_vision;
using namespace cv;
int main(){
    int i;
    Detector t;
//    t.setDetector("CambriconYoloDetector", "CambriconYoloDetector", PYTHON, "");
//    cv::Mat img = cv::imread("/home/de/data/caffe_show/8.png");
//    for( int i= 0; i < 1000;i++){
//        t.detectionOnce(cv::Mat(), img);
//        cout << i <<endl;
//        sleep(0.1);
//    }
    std::cout << "************"<<std::endl;
    t.setDetector("TOD", "coke", CPP, "/home/de/work/hirop/config/TODconfig.yaml");
    cv::Mat img ,xmlDepth;
    FileStorage fs("/home/de/linemod测试样本/data-test/depth/depthTest1.xml", FileStorage::READ);
    fs["depth"] >> xmlDepth;
    fs.release();
    img = cv::imread("/home/de/linemod测试样本/data-test/JPEGImages/ColorZero1.png");
    sleep(1);
    t.detectionOnce(xmlDepth, img);

    return 0;
}
