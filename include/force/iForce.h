#ifndef IGRIPPER_H
#define IGRIPPER_H

#pragma once

#include <yaml-cpp/yaml.h>
#include "hplugin.h"
#include <vector>
#include <force/forceExport.h>
#include <string>
namespace hirop_force {

class IForce{
public:

        /**
         * @brief 解析私有参数
         * @return
         */
        virtual int parseConfig(YAML::Node& ) = 0;

        /**
         * @brief setInputForce 输入力矩参数
         * @return
         */
        virtual void setInputForceBias(std::vector<double> &value) = 0;

        /**
         * @brief setInputRobotPose 输入机器人的笛卡尔坐标
         * @return
         */
        virtual void setInputRobotCartPose(std::vector<double> &value) = 0;

        /**
         * @brief compute 算法运算
         * @return
         */
        virtual int compute() = 0;

        /**
         * @brief getResult 获取力控运算的输出结果
         * @return
         */
        virtual int getResult(std::vector<double> &) = 0;


        /**
         * @brief getName  获取算法插件的名称
         * @param name
         * @return
         */
        virtual std::string  getName() = 0;


        /**
         * @brief printInfo 输出打印信息
         * @return
         */
        virtual int printInfo() = 0;
protected:
    // 保存器的字符描述
    std::string name;

    // 其算法实现类别 C++ / Python
    hirop_force::ENTITY_TYPE entityType;
};

}
H_DECLARE_INTERFACE(hirop_force::IForce,"IForceV1.0")
#endif
