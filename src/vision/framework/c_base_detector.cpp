#include "vision/c_base_detector.h"

using namespace hirop_vision;

CBaseDetector::CBaseDetector(std::string name, bool isMultiDetector){
    this->name = name;
    this->entityType = CPP;
    this->_isMultiDetector = isMultiDetector;
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

int CBaseDetector::isMultiDetector(){
    return _isMultiDetector;
}

int CBaseDetector::parseConfig(const YAML::Node &node){
    return 0;
}
