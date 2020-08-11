#pragma once

#include "iForce.h"

namespace hirop_force {

class IForceLoader{

public:
    /**
     * @brief loadForcePlugin 根据名称加载
     * @param
     * @return
     */
    virtual IForce *loadForcePlugin(std::string& ) = 0;


    /**
     * @brief getGeneratorList
     * @param
     */
    virtual void getForcePluginList(std::vector<std::string>& ) = 0;

};

}
