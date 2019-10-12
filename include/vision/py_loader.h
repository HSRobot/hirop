#ifndef __PY_LOADER_H__
#define __PY_LOADER_H__

#include <iostream>

#include "itrainer.h"
#include "idetector.h"
#include "iloader.h"

/**
 * @brief   PY_DETECTOR_REGEX   通过文件名获取识别器名称的正则表达式
 * \\w+     表示有1个或1个以上的字符串
 * Detector  表示识别器名称最后必须由Detector组成
 */
#define PY_DETECTOR_REGEX "\\w+Detector(?=\\.py\\b)"

namespace hirop_vision{

/**
 * @brief       python实例加载器，采用单列模式
 * @author      XuKunLin
 * @date        2019-04-22
 */

class PyLoader : public ILoader{

public:
    /**
     * @brief   加载识别器
     * @param[in] detectorName 识别器名称
     * @return  识别器实例指针
     */
    IDetector *loadDetector(std::string detectorName);

    /**
     * @brief loadTrainer   加载训练器
     * @param trainerName   训练器的名称
     * @return              训练器实例的指针
     */
    ITrainer *loadTrainer(std::string trainerName);

    /**
     * @brief getDetectorList   获取系统中可用的识别器列表
     * @param detectorList[out] 识别器列表
     */
    void getDetectorList(std::vector<std::string> &detectorList);

    /**
     * @brief       获取当前系统中可用的训练器列表
     * @param[out]  trainerList 系统中可用的训练器列表
     */
    void getTrainerList(std::vector<std::string> &trainerList);

    /**
     * @brief   获取pyloader的是列
     * @return  返回加载器实例指针
     */
    static PyLoader *getPyLoader();

private:
    /**
     * @brief   构造函数
     */
    PyLoader();

    /**
     * @brief   初始化python解析器系统路径
     * @return  0成功 -1失败
     */
    int initSysPath();

public:
    static std::string PATH;
    static PyLoader *instance;
};

}

#endif
