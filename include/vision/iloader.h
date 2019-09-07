#pragma once

#include "itrainer.h"
#include "idetector.h"

namespace hirop_vision {

class ILoader{

public:
    /**
     * @brief loadTrainer   根据名称加载训练器
     * @param trainerName   训练器的名称
     * @return  训练器实例的指针
     */
    virtual ITrainer *loadTrainer(std::string trainerName) = 0;

    /**
     * @brief loadDetector  根据名称加载识别器
     * @param detectorName  识别器的名称
     * @return              识别器实例的指针
     */
    virtual IDetector *loadDetector(std::string detectorName) = 0;

    /**
     * @brief getDetectorList   获取识别器的列表
     * @param detectorList      识别的列表
     */
    virtual void getDetectorList(std::vector<std::string> &detectorList) = 0;

    /**
     * @brief       获取当前系统中可用的训练器列表
     * @param[out]  trainerList 系统中可用的训练器列表
     */
    virtual void getTrainerList(std::vector<std::string> &trainerList) = 0;
};

}
