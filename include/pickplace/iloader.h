#pragma once

#include "igenerator.h"
#include "ipickplace.h"

namespace hirop_pickplace {

class ILoader{

public:
    /**
     * @brief loadGenerator 根据名称加载生成器
     * @param  生成器的名称
     * @return  生成器器实例的指针
     */
    virtual IGenerator *loadGenerator(std::string) = 0;

    /**
     * @brief loadPickPlace  根据名称加载执行器器
     * @param  执行器的名称
     * @return             　 执行器实例的指针
     */
    virtual IPickPlace *loadPickPlace(std::string) = 0;

    /**
     * @brief getGeneratorList   获取生成器的列表
     * @param  　　　　　　　　　  　生成器的列表
     */
    virtual void getGeneratorList(std::vector<std::string>& ) = 0;

    /**
     * @brief getActuatorList    获取当前系统中可用的执行器器列表
     * @param[out]  　　　　　　　　系统中可用的执行器器列表
     */
    virtual void getActuatorList(std::vector<std::string>& ) = 0;
};

}
