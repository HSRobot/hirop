#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "vision/c_base_detector.h"
#include "hpluginloader.h"
#include "linemod_detector_Tool.h"
#include "tensorflowCCVisionLib.h"
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif

using namespace std;
using namespace hirop_vision;
/**
 * 
 * 移值tensorflow object detection 模块
 * 
 */
using namespace hirop_vision;
class TODDetector:public CBaseDetector{

public:
    TODDetector();
    ~TODDetector();
   /**
     * @brief   实现具体的识别功能，需要具体的识别器实现
     * @return void
     */
    virtual int detection();

    //    /**
    //     * @brief   解析识别器私有训练器配置，具体由相关识别器实现
    //     * @return void
    //     */
    //    virtual int parseConfig()  ;

    /**
     * @brief   加载相关识别器识别时需要的数据
     * @return void
     */
    virtual int loadData(const std::string path, const std::string objectName)  ;

    /**
     * @brief   传递识别需要的彩色图片数据
     * @return void
     */
    virtual void setColorImg(const cv::Mat &inputImg);

    /**
    * @brief   传递识别需要的深度图片数据
    * @return void
    */
    virtual void setDepthImg(const cv::Mat &inputImg);

    //    /**
    //     * @brief   获取识别过程中的预览图片
    //     * @return void
    //     */
    //    virtual void getImg()  ;

    /**
     * @brief  获取图像识别过程中的结果
     * @return 0 成功 -1 失败
     */
    virtual int getResult(std::vector<pose> &poses)  ;

    /**
     * @brief  获取识别器的名称
     * @return 0 成功 -1 失败
     */
    virtual int getName(std::string &name)  ;

    /**
     * @brief   是否支持多物体检测
     * @return  1 支持 0 不支持
     */
    virtual int isMultiDetector()  ;


    /**
     * @brief      获取当前训练器的实现实体
     * @return      返回训练器的实现实体类别
     */
    virtual ENTITY_TYPE getEntityType()  ;
private:
    int initParam();
    void ScalarPoint(const Point& p01, const Point& p02, Point& p11, Point& p12);
private:
    TensorflowCCVisoin *TCCVSION;
    vector<float> outPose;
    std::vector<pose> outposedd;
    string name;
    int index;
    cv::Rect depthRect;
    int ColorWidth, ColorHeight;
    cv::Mat DepthImg ,ColorImg, ColorShow, DepthShow;
    string ThisDetectionName;
private:
    // opencv linemod对象指针
    // cv::linemod::Detector *detector_;
    LinemodDetector *LineDeTool;


};

H_DECLARE_PLUGIN(IDetector)
