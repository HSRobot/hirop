#include "simaple_detector.h"

#include <opencv2/highgui.hpp>

SimapleDetector::SimapleDetector():CBaseDetector("SimapleDetector", false){
    std::cout << "SimapleDetector init "<<std::endl;
}

int SimapleDetector::getResult(std::vector<pose> &poses){
	return 0;
}

int SimapleDetector::detection(){
//    cv::imwrite("/home/de/test.jpg", _colorImg);
    std::cout << "pointcloud2_ptr size: "<<pointcloud2_ptr.data.size()<<std::endl;
    return 0;
}

void SimapleDetector::setColorImg(const cv::Mat &inputImg){
    _colorImg = inputImg;
}

void SimapleDetector::setPointCloud(const pcl::PCLPointCloud2 &pointcloud2_ptr)
{
    this->pointcloud2_ptr = pointcloud2_ptr;
}

void SimapleDetector::setDepthImg(const cv::Mat &inputImg){

}

int SimapleDetector::loadData(const std::string path, const std::string objectName){
    return 0;
}

H_EXPORT_PLUGIN(SimapleDetector, "SimapleDetector", "1.0")
