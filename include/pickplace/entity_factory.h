#ifndef __ENTITY_FACTORY_H__
#define __ENTITY_FACTORY_H__

#include <iostream>
#include <map>

#include "igenerator.h"
#include "ipickplace.h"

namespace hirop_pickplace {

/**
 * @brief       实例加载工厂类
 * @author      XuKunLin
 * @date        2019-04-02
 */
class EntityFactory{

public:
    /**
     * @brief       根据名称获取生成器
     * @param[in]   name，生成器的名称
     * @param[out]  trainer，返回生成器的指针
     * @return 0 成功 -1 失败
     */
    int getGenerator(const std::string &name, IGenerator *generator);

    /**
     * @brief       根据名称获取执行器
     * @param[in]   name，执行器的名称
     * @param[out]  detector，返回执行器的指针
     * @return 0 成功 -1 失败
     */
    int getPickPlacer(const std::string &name, IPickPlace *pickPlace);

private:
    /**
     * @brief trainers  当前系统中已加载的生成器
     */
    std::map<std::string, IGenerator *> generator;

    /**
     * @brief detectors 当前系统中已加载的执行器
     */
    std::map<std::string, IPickPlace *> pickPlace;

};

}


#endif
