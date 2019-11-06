#ifndef IGRIPPER_H
#define IGRIPPER_H

#pragma once

#include <yaml-cpp/yaml.h>
#include "hplugin.h"
#include "gripper.h"

namespace hirop_gripper {

class IGripper{
public:

    /**
     * @brief 解析私有参数
     * @return
     */
    virtual int parseConfig(YAML::Node& ) = 0;

    /**
     * @brief connectGripper 连接夹爪
     * @return
     */
    virtual int connectGripper() = 0;

    /**
     * @brief isConnectGripper 夹爪是否连接
     * @return
     */
    virtual bool isConnectGripper() = 0;

    /**
     * @brief disConnectGripper 关闭夹爪
     * @return
     */
    virtual int disConnectGripper() = 0;

    /**
     * @brief openGripper 打开夹爪
     * @return
     */
    virtual int openGripper() = 0;

    /**
     * @brief closeGripper 关闭夹爪
     * @return
     */
    virtual int closeGripper() = 0;

    /**
     * @brief stopGripper 夹爪急停
     * @return
     */
    virtual int stopGripper() = 0;

    virtual int getName(std::string &name) = 0;

    virtual ~IGripper(){}

protected:
    // 保存器的字符描述
    std::string name;

    // 其算法实现类别 C++ / Python
    hirop_gripper::ENTITY_TYPE entityType;
};

}
H_DECLARE_INTERFACE(hirop_gripper::IGripper,"IGripperv1.0")
#endif // IGRIPPER_H
