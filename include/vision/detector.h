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
#ifndef __DETCTOR_H__
#define __DETCTOR_H__


#include "vision.h"
#include "detection_listener.h"

#include <boost/thread.hpp>
#include "py_loader.h"
#include "cpp_loader.h"

namespace hirop_vision {

/**
 * @brief           hirop_vision相关detector的接口类
 *
 *  检测器对于实现语言有两大类别，C++和Python。对于其能力也分为两大类别，多物体识别器和单物体识别器
 *  对于多物体识别器，在整个程序生命周期，只拥有一个检测器的实例，其配置是通用的。
 *  而对于单物体识别器，在整个程序生命周期中，对于不同物体均有一个实例，其每个实例的配置是独立的。
 *
 * @author          XuKunLin
 * @date            2019-03-20
 * @todo            1，当传递检测器名称时，检测检测器是否存在
 *                  2，当传递物体名称给检测器时，检测是否有该物体。
 *                  3，支持多物体、多训练器
 *                  4，增加状态枚举
 *                  5，增加析构函数，进行内存回收
 */
class Detector{

    /**
 * @brief       检测完成后的回调函数原型
 * @param       [detectorName] 检测器的名称
 * @param       [pose]         检测结果，姿态。 基于相机坐标的位姿，四元数表示法。
 * @return      无返回
 */
    typedef void (*CBFUN)(std::string detectorName, pose p);

public:

    /**
     * @brief       构造函数
     */
    Detector();

    /**
     * @brief       设置检测器的名称和类别
     * @param[in] name，检测器的名称
     * @param[in] type，检测器的类别
     * @param[in] objectName，检测物体的名称
     * @param[in] configFile，配置文件的路径
     * @return 0 成功 -1 失败
     */
    int setDetector(const std::string &name, const std::string &objectName, ENTITY_TYPE type, \
                    std::string configFile = "");

    /**
     * @brief   开始进行物体识别，只检测一次，检测成功后即返回
     * @param [objectName]      指定需要识别物体的名称
     * @param [detectorName]   指定识别器名称
     * @return
     *          0  开始识别成功
     *          -1 开始识别失败
     */
    int detectionOnce(const cv::Mat &depthImg, const cv::Mat &colorImg);

    /**
     * @brief       开始进行物体识别，循环检测
     * @param
     * @return
     */
    int detection(std::string objectName, std::string detectorName,  const cv::Mat &depthImg, const cv::Mat &colorImg);

    /**
     * @brief       设置当识别状态监听者实例
     * @param [listener] 监听者实例
     * @return  0 成功 1 失败
     */
    int setOnStateChangeCallback(DetectStateListener *listener);

private:

    /**
     * @brief       执行具体的识别工作，将会已线程的方式被调用
     * @param [objName] 被检测的物体名称
     * @param [detector] 即将执行识别工作的识别器实例
     * @return
     */
    int __detection(bool loop);


private:
    // 状态监听者实例
    DetectStateListener *listener;

    // 识别线程
    boost::thread *detectionThr;

    // 识别的配置信息
    std::string objectName;
    std::string detectorName;

    CppLoader *cppLoader;

    PyLoader *pyLoader;

    IDetector *detectorPtr;

    /**
     * @brief cppDetectors C++多物体识别器缓存
     */
    std::map<std::string, IDetector*> cppDetectors;

    /**
     * @brief pyDetectors python多物体识别器缓存
     */
    std::map<std::string, IDetector*> pyDetectors;

    /**
     * @brief cppSingleDetectors C++单物体识别器缓存
     */
    std::map<std::string, std::map<std::string, IDetector*> > cppSingleDetectors;

    /**
     * @brief pySingleDetectors python单物体识别器缓存
     */
    std::map<std::string, std::map<std::string, IDetector*> > pySingleDetectors;
};

}

#endif
