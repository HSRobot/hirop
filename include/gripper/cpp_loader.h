#ifndef LOADER_H
#define LOADER_H

#include <boost/filesystem.hpp>

#include "igripper.h"
#include "iloader.h"

#include <hpluginloader.h>

/**
 * @brief   GRIPPER_REGEX   通过文件名获取夹爪名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示夹爪名称最后必须由Gripper组成
 */
#define GRIPPER_REGEX "(?<=lib)\\w+(?<=Gripper)"

#define LIB_SERACH_PATH ""

/**
 * @brief   LIB_SERACH_PATH_ENV
 *          获取插件搜索路径的环境变量名称
 */
#define LIB_SERACH_PATH_ENV "GRIPPER_PLUGIN_PATH"

namespace hirop_gripper{

class CppLoader : public ILoader{

public:
    CppLoader();

    /**
     * @brief       加载相关的夹爪
     * @param    [trainerName] 输入，生成器的名称
     * @return      返回相关生成器的指针
     */
    IGripper *loadGripper(std::string gripperName);

    /**
     * @brief 获取夹爪列表
     * @param generatorList，生成器表
     */
    void getGripperList(std::vector<std::string>& gripperList);

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
