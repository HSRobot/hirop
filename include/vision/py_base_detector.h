#ifndef __PY_BASE_DETECTOR_H__
#define __PY_BASE_DETECTOR_H__

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "idetector.h"

namespace  hirop_vision{

/**
 * @brief       python实例的基类，也是代理类，通过该类访问具体的python实例
 * @author      XuKunLin
 * @date        2019-04-22
 */

class PyBaseDetector:public IDetector{

public:
    /**
     * @brief   构造函数
     * @param[in] pClass，Python文件中的类对象
     * @param[in] detectorName，识别器的名称
     */
    PyBaseDetector(PyObject *pClass, std::string detectorName);

    /**
     * @brief   析构函数
     */
    ~PyBaseDetector();

    /**
     * @brief   解析识别器私有训练器配置，具体由相关识别器实现
     * @return void
     */
    int parseConfig(const YAML::Node &node);

    /**
     * @brief   调用该函数后，检测器便开始进行物体检测
     * @return  0 检测成功 -1 检测失败
     */
    int detection();

    /**
     * @brief   加载检测器相关的参数
     * @param[in] path，配置文件的路径
     * @param[in] objectName，被检测的物体名称
     * @return  0 加载成功 -1 加载失败
     */
    int loadData(const std::string path, const std::string objectName);

    /**
     * @brief   设置彩色图像
     * @param[in] inputImg，彩色图像数据
     * @return 0 成功 -1 失败
     */
    void setColorImg(const cv::Mat &inputImg);

    /**
     * @brief   设置深度图像
     * @param[in] inputImg，深度图像数据
     * @return 0 成功 -1 失败
     */
    void setDepthImg(const cv::Mat &inputImg);

    /**
     * @brief   获取识别得到的位姿
     * @param[out] poses，识别到的物体和对应的位姿
     * @return 0 成功 -1 失败
     */
    int getResult(std::vector<pose> &poses);

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

    /**
     * @brief   是否支持多物体检测
     * @return  1 支持 0 不支持
     */
    int isMultiDetector();

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
