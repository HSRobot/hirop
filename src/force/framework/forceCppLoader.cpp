#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <force/forceCppLoader.h>
#include "utils/fs_helper.h"
#include <hpluginloader.h>
using namespace hirop_force;

forceCppLoader::forceCppLoader()
{
    const char *env;

    this->loader = HPluginLoader::getLoader();

    env = getenv(LIB_SERACH_PATH_ENV);

    if(env != NULL){

        libSerachPath = env;

        std::vector<std::string> paths;
        boost::split(paths, libSerachPath, boost::is_any_of(":"));

        /**
         * @brief 从环境变量中获取库的搜索路径，支持多路径
         */
        BOOST_FOREACH(std::string path, paths){
            libSerachPaths.push_back(path + "/");
        }

    }
    /**
     * @brief 支持从当前路径加载插件
     */
    libSerachPaths.push_back("./");

    /**
     * 支持从LD_LIBRARY_PATH环境变量路径加载插件
     */
    //libSerachPaths.push_back("");
}

IForce* forceCppLoader::loadForcePlugin(std::string &forcePluginName){

    HPlugin *plugin;
    loader->setPaths(libSerachPaths);
    plugin = loader->load(forcePluginName);
    if(plugin == NULL)
        return NULL;
    return plugin->instance<IForce>();
}

void forceCppLoader::getForcePluginList(std::vector<std::string> &forceList)
{
    return FSHelper::filterFilesByRegx(libSerachPaths, forceList, FORCE_REGEX);
}


