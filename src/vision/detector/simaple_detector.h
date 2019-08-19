#pragma once

#include <hpluginloader.h>
#include <hirop/vision/c_base_detector.h>

using namespace hirop_vision;

class SimapleDetector: public CBaseDetector{

public:

    SimapleDetector();

    /**
     * @brief   实现具体的识别功能
     * @return  0 成功 -1 失败
     */
    int detection();

    /**
     * @brief   加载相关的训练结果
     * @param   [objectName] 需要识别的物体
     * @return  0 成功 -1 失败
     */
    int loadData(const std::string path, const std::string objectName);

    /**
     * @brief   获取识别的结果
     * @param[out] poses， 保存识别的结果
     * @return  0 成功 -1 失败
     */
    int getResult(std::vector<pose> &poses);

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setDepthImg(const cv::Mat &inputImg);

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setColorImg(const cv::Mat &inputImg);

private:

    cv::Mat _colorImg;

};

H_DECLARE_PLUGIN(IDetector)
