#include "vision/c_base_detector.h"

using namespace hirop_vision;

CBaseDetector::CBaseDetector(std::string name, bool isMultiDetector){
    this->name = name;
    this->entityType = CPP;
    this->_isMultiDetector = isMultiDetector;
    this->havePreviewImage = false;
}

int CBaseDetector::getName(std::string &name){
    name = this->name;
    return 0;
}

ENTITY_TYPE CBaseDetector::getEntityType(){
    return this->entityType;
}

void CBaseDetector::setColorImg(const cv::Mat &inputImg){

}

void CBaseDetector::setDepthImg(const cv::Mat &inputImg){

}

void CBaseDetector::setPointCloud(const pcl::PCLPointCloud2& pointcloud2_ptr)
{

}

int CBaseDetector::isMultiDetector(){
    return _isMultiDetector;
}

int CBaseDetector::parseConfig(const YAML::Node &node){
    return 0;
}

int CBaseDetector::getPreImg(cv::Mat &preImg){
    return -1;
}
