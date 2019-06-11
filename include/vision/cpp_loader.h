#ifndef __CPP_LOADER_H__
#define __CPP_LOADER_H__

#include "itrainer.h"
#include "idetector.h"

#include "hplugin/hpluginloader.h"

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

private:
    HPluginLoader *loader;
};

}

#endif
