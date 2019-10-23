#ifndef LOADER_H
#define LOADER_H

#include <boost/filesystem.hpp>

#include "igenerator.h"
#include "ipickplace.h"
#include "iloader.h"

#include <hpluginloader.h>

/**
 * @brief   GENERATOR_REGEX   通过文件名获取识别器名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示生成器名称最后必须由Generator组成
 */
#define GENERATOR_REGEX "(?<=lib)\\w+(?<=Generator)"

/**
 * @brief   ACTUATOR_REGEX   通过文件名获取训练器名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示执行器名称最后必须由Actuator组成
 */
#define ACTUATOR_REGEX "(?<=lib)\\w+(?<=Actuator)"

#define LIB_SERACH_PATH ""

/**
 * @brief   LIB_SERACH_PATH_ENV
 *          获取插件搜索路径的环境变量名称
 */
#define LIB_SERACH_PATH_ENV "PICKPLACE_PLUGIN_PATH"

namespace hirop_pickplace{

class CppLoader : public ILoader{

public:
    CppLoader();

    /**
     * @brief       加载相关的生成器
     * @param    [trainerName] 输入，生成器的名称
     * @return      返回相关生成器的指针
     */
    IGenerator *loadGenerator(std::string generatorName);

    /**
     * @brief       加载相关的执行器
     * @param    [trainerName] 输入，执行器的名称
     * @return      返回相关执行器的指针
     */
    IPickPlace *loadPickPlace(std::string PickPlaceName);

    /**
     * @brief 获取生成器列表
     * @param generatorList，生成器表
     */
    void getGeneratorList(std::vector<std::string>& generatorList);

    /**
     * @brief 获取执行器列表
     * @param actuatorList，执行器表
     */
    void getActuatorList(std::vector<std::string>& actuatorList);

private:
     /**
     * @brief 具体加载器
     */
    HPluginLoader *loader;

    /**
     * @brief libSerachPath 库的搜索路径
     */
    std::string libSerachPath;

    std::vector<std::string> libSerachPaths;
};

}
#endif // LOADER_H
