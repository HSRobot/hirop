/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 *
 * Author: Kunlin Xu <1125290220@qq.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 */
#ifndef __IDETERCTOR_H__
#define __IDETERCTOR_H__

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include <hplugin.h>
#include "vision.h"


namespace hirop_vision{

/**
 * @brief       具体识别算法识别器基类
 * @author      XuKunLin
 * @date        2019-03-19
 */
class IDetector{

public:
    /**
     * @brief   实现具体的识别功能，需要具体的识别器实现
     * @return void
     */
    virtual int detection() = 0;

    /**
     * @brief   解析识别器私有训练器配置，具体由相关识别器实现
     * @return void
     */
    virtual int parseConfig(const YAML::Node &node) = 0;

    /**
     * @brief   加载相关识别器识别时需要的数据
     * @return void
     */
    virtual int loadData(const std::string path, const std::string objectName) = 0;

    /**
     * @brief   传递识别需要的彩色图片数据
     * @return void
     */
    virtual void setColorImg(const cv::Mat &inputImg) = 0;

    /**
     * @brief   传递识别需要的深度图片数据
     * @return void
     */
    virtual void setDepthImg(const cv::Mat &inputImg) = 0;

    /**
     * @brief   获取识别过程中的预览图片
     * @param perImg[out]   预览图片
     * @return      0 正常获取到预览图片 -1 反之
     */
    virtual int getPreImg(cv::Mat &preImg) = 0;

    /**
     * @brief  获取图像识别过程中的结果
     * @return 0 成功 -1 失败
     */
    virtual int getResult(std::vector<pose> &poses) = 0;

    /**
     * @brief  获取识别器的名称
     * @return 0 成功 -1 失败
     */
    virtual int getName(std::string &name) = 0;

    /**
     * @brief   是否支持多物体检测
     * @return  1 支持 0 不支持
     */
    virtual int isMultiDetector() = 0;


    /**
     * @brief      获取当前训练器的实现实体
     * @return      返回训练器的实现实体类别
     */
    virtual ENTITY_TYPE getEntityType() = 0;

    /**
     * @brief havePreImg    返回是否有预览图片
     * @return
     */
    virtual bool havePreImg() {return havePreviewImage;}

protected:

    /**
     * @brief name      识别器的名称
     */
    std::string name;

    /**
     * @brief havePreviewImage  该实例是否会生成预览图片
     */
    bool havePreviewImage;

    /**
     * @brief entityType    实例的实现类别 PYTHON 或 C++
     */
    hirop_vision::ENTITY_TYPE entityType;

    /**
     * @brief _isMultiDetector 是否支持多物体识别
     */
    bool _isMultiDetector;

};

}
H_DECLARE_INTERFACE(hirop_vision::IDetector, "IDetector/V1.0")

#endif
