#ifndef __PY_BASE_DETECTOR_H__
#define __PY_BASE_DETECTOR_H__

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <yaml-cpp/yaml.h>
#include "ipickplace.h"

namespace  hirop_pickplace{

/**
 * @brief       python实例的基类，也是代理类，通过该类访问具体的python实例
 * @author      ShenCanJun
 * @date        2019-04-22
 */

class PyBasePickPlace:public IPickPlace{

public:
    /**
     * @brief   构造函数
     * @param[in] pClass，Python文件中的类对象
     * @param[in] detectorName，识别器的名称
     */
    PyBasePickPlace(PyObject *pClass, std::string pickplaceName);

    /**
     * @brief   析构函数
     */
    ~PyBasePickPlace();

    /**
     * @brief   解析识别器私有训练器配置，具体由相关识别器实现
     * @return void
     */
    int parseConfig(const YAML::Node &node);

    /**
     * @brief   设置抓取的机器人位姿
     * @param[in] 机器人位姿
     * @return 0 成功 -1 失败
     */
    int setPickPose(Pose pose);

    /**
     * @brief   设置放置的机器人位姿
     * @param[in] 机器人位姿
     * @return 0 成功 -1 失败
     */
    int setPlacePose(Pose pose);

    /**
     * @brief   执行抓取
     * @return 0 成功 -1 失败
     */
    int pick();

    /**
     * @brief   执行放置
     * @return 0 成功 -1 失败
     */
    int place();

    /**
     * @brief   获取识别器的名称
     * @param[out] name，返回识别器的名称
     * @return 0 成功 -1 失败
     */
    int getName(std::string &name);


    /**
     * @brief   获取识别器的类别
     * @return  返回识别器的类别
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
