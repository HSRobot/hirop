#pragma once

#include "igripper.h"

namespace hirop_gripper {

class ILoader{

public:
    /**
     * @brief loadGripper 根据名称加载夹爪
     * @param  夹爪的名称
     * @return  夹爪实例的指针
     */
    virtual IGripper *loadGripper(std::string) = 0;


    /**
     * @brief getGeneratorList   获取夹爪的列表
     * @param  　　　　　　　　　  　夹爪的列表
     */
    virtual void getGripperList(std::vector<std::string>& ) = 0;

};

}
