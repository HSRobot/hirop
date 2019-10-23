#ifndef __PY_BASE_GENERATOR_H__
#define __PY_BASE_GENERATOR_H__

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <yaml-cpp/yaml.h>
#include "igenerator.h"

namespace  hirop_pickplace{

/**
 * @brief       python实例的基类，也是代理类，通过该类访问具体的python实例
 * @author      ShenCanJun
 * @date        2019-04-22
 */

class PyBaseGenerator:public IGenerator{

public:
    /**
     * @brief   构造函数
     * @param[in] pClass，Python文件中的类对象
     * @param[in] detectorName，识别器的名称
     */
    PyBaseGenerator(PyObject *pClass, std::string generatorName);

    /**
     * @brief   析构函数
     */
    ~PyBaseGenerator();

    /**
     * @brief   解析识别器私有配置，具体由相关识别器实现
     * @return void
     */
    int parseConfig(const YAML::Node &node);

    /**
     * @brief 设置抓取点位
     * @param[in] 抓取的目标点位
     * @return 0 成功 -1 失败
     */
    int setPickPose(Pose pose);

    /**
     * @brief   设置放置点位
     * @param[in] 放置点位
     * @return 0 成功 -1 失败
     */
    int setPlacePose(Pose pose);

    /**
     * @brief   获取生成后的机器人位姿
     * @param[out] pose，生成的机器人位姿
     * @return 0 成功 -1 失败
     */
    int getResultPickPose(Pose pose);

    int getResultPlacePose(Pose pose);

    /**
     * @brief   获取生成器的名称
     * @param[out] name，返回生成器的名称
     * @return 0 成功 -1 失败
     */
    int getName(std::string &name);

    /**
     * @brief   获取生成器的类别
     * @return  返回生成器的类别
     */
    ENTITY_TYPE getEntityType();

private:
#ifdef WITH_PYTHON3
    int initNump();
#else
    void initNump();
#endif

private:
    PyObject *pClass;
    PyObject *pClassInstance;
    PyObject  *pDict;
};
}


#endif
