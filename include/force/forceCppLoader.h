#pragma once

/**
 * @brief   GRIPPER_REGEX   通过文件名获取夹爪名称的正则表达式
 * (?<=lib) 表示lib参与正则表达式的匹配，但是获取匹配结果时不获取lib
 * \\w+     表示有1个或1个以上的字符串
 * Trainer  表示夹爪名称最后必须由Gripper组成
 */
#define FORCE_REGEX "(?<=lib)\\w+(?<=Force)"

#define LIB_SERACH_PATH ""

/**
 * @brief   LIB_SERACH_PATH_ENV
 *          获取插件搜索路径的环境变量名称
 */
#define LIB_SERACH_PATH_ENV "FORCE_PLUGIN_PATH"
#include <boost/filesystem.hpp>

#include <force/IForceLoader.h>
namespace hirop_force {

class forceCppLoader :public IForceLoader{

public:
    /**
     * @brief 构造函数
     * @param
     */
    forceCppLoader();

    /**
     * @brief       获取配置文件中夹爪的名称
     * @param   [trainerName] 输出，生成器名称
     * @return
     *          0 成功
     *          1 失败
     */
    IForce* loadForcePlugin(std::string& forcePluginName);


    /**
     * @brief getForcePluginList
     * @param forceList
     */
    void getForcePluginList(std::vector<std::string> &forceList);

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
