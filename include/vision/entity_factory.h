#ifndef __ENTITY_FACTORY_H__
#define __ENTITY_FACTORY_H__

#include <iostream>
#include <map>

#include "itrainer.h"
#include "idetector.h"

namespace hirop_vision {

/**
 * @brief       实例加载工厂类
 * @author      XuKunLin
 * @date        2019-04-02
 */
class EntityFactory{

public:
    /**
     * @brief       根据名称获取训练器
     * @param[in]   name，训练器的名称
     * @param[out]  trainer，返回训练器的指针
     * @return 0 成功 -1 失败
     */
    int getTrainer(const std::string &name, ITrainer *trainer);

    /**
     * @brief       根据名称获取识别器
     * @param[in]   name，识别器的名称
     * @param[out]  detector，返回识别器的指针
     * @return 0 成功 -1 失败
     */
    int getDetector(const std::string &name, IDetector *detector);

private:
    /**
     * @brief trainers  当前系统中已加载的训练器
     */
    std::map<std::string, ITrainer *> trainers;

    /**
     * @brief detectors 当前系统中已加载的识别器
     */
    std::map<std::string, IDetector *> detectors;

};

}


#endif
