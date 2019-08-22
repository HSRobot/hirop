#include "simaple_detector.h"

#include <opencv2/highgui.hpp>

SimapleDetector::SimapleDetector():CBaseDetector("LinemodDetector", false){

}

int SimapleDetector::getResult(std::vector<pose> &poses){
	return 0;
}

int SimapleDetector::detection(){
    cv::imwrite("/home/de/test.jpg", _colorImg);
    return 0;
}

void SimapleDetector::setColorImg(const cv::Mat &inputImg){
    _colorImg = inputImg;
}

void SimapleDetector::setDepthImg(const cv::Mat &inputImg){

}

int SimapleDetector::loadData(const std::string path, const std::string objectName){
    return 0;
}

H_EXPORT_PLUGIN(SimapleDetector, "SimapleDetector", "1.0")
