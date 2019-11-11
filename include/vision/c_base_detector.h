#ifndef __C_BASE_DETECTOR_H__
#define __C_BASE_DETECTOR_H__

#include "idetector.h"

namespace hirop_vision {

class CBaseDetector:public IDetector{

public:
    CBaseDetector(std::string name, bool isMultiDetector);

    int getName(std::string &name);

    ENTITY_TYPE getEntityType();

    void setColorImg(const cv::Mat &inputImg);

    void setDepthImg(const cv::Mat &inputImg);

    void setPointCloud(const pcl::PCLPointCloud2 &pointcloud2_ptr);

    int isMultiDetector();

    int parseConfig(const YAML::Node &node);

    virtual int getPreImg(cv::Mat &preImg);
};

}
#endif
