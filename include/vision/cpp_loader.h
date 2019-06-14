#ifndef __CPP_LOADER_H__
#define __CPP_LOADER_H__

#include <boost/filesystem.hpp>

#include "itrainer.h"
#include "idetector.h"

#include "hplugin/hpluginloader.h"


/**
 * @brief   DETECTOR_REGEX   通过文件名获取识别器名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示识别器名称最后必须由Trainer组成
 */
#define DETECTOR_REGEX "(?<=lib)\\w+Detector"

/**
 * @brief   DETECTOR_REGEX   通过文件名获取训练器名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示训练器名称最后必须由Trainer组成
 */
#define TRAINER_REGEX "(?<=lib)\\w+Trainer"

#define LIB_SERACH_PATH "./"

namespace hirop_vision{

class CppLoader{

public:
    CppLoader();

    /**
     * @brief       加载相关的训练库
     * @param    [trainerName] 输入，训练库的名称
     * @return      返回相关训练库的指针
     */
    ITrainer *loadTrainer(std::string trainerName);

    /**
     * @brief       加载相关的识别库
     * @param    [trainerName] 输入，识别库的名称
     * @return      返回相关识别库的指针
     */
    IDetector *loadDetector(std::string trainerName);

    /**
     * @brief       获取当前系统中可用的检测器列表
     * @param[out]  detectorList 系统中可用的检测器列表
     */
    void getDetectorList(std::vector<std::string> &detectorList);

    /**
     * @brief       获取当前系统中可用的训练器列表
     * @param[out]  trainerList 系统中可用的训练器列表
     */
    void getTrainerList(std::vector<std::string> &trainerList);

private:
    HPluginLoader *loader;

    /**
     * @brief libSerachPath 库的搜索路径
     */
    std::string libSerachPath;
};

}

#endif
