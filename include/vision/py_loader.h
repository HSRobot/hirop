#ifndef __PY_LOADER_H__
#define __PY_LOADER_H__

#include <iostream>

#include "itrainer.h"
#include "idetector.h"

namespace hirop_vision{

/**
 * @brief       python实例加载器，采用单列模式
 * @author      XuKunLin
 * @date        2019-04-22
 */

class PyLoader{

public:
    /**
     * @brief   加载识别器
     * @param[in] detectorName 识别器名称
     * @return  识别器实例指针
     */
    IDetector *loadDetector(const std::string &detectorName);

    //    ITrainer *loadTrainer(const std::string &trainerName);
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
    const static std::string PATH;
    static PyLoader *instance;
};

}

#endif
