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

    int isMultiDetector();

private:
};

}

#endif
