#include "vision/detector.h"
#include "vision/vision.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "pcl/PCLPointCloud2.h"
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
    t.setDetector("Simaple", "Simaple", CPP, "");
    cv::Mat img ,xmlDepth;
    pcl::PointCloud<pcl::PointXYZ> temp;
//    t.detectionOnce(xmlDepth, img, );

    return 0;
}
