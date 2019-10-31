#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "pickplace/cpp_loader.h"

#include "utils/fs_helper.h"

using namespace hirop_pickplace;

CppLoader::CppLoader()
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

IGenerator* CppLoader::loadGenerator(std::string generatorName){

    HPlugin *plugin;
    loader->setPaths(libSerachPaths);
    plugin = loader->load(generatorName);
    if(plugin == NULL)
        return NULL;
    return plugin->instance<IGenerator>();
}

IPickPlace* CppLoader::loadPickPlace(std::string PickPlaceName){
    HPlugin *plugin;
    loader->setPaths(libSerachPaths);
    plugin = loader->load(PickPlaceName);
    if(plugin == NULL)
        return NULL;
    return plugin->instance<IPickPlace>();
}

void CppLoader::getGeneratorList(std::vector<std::string> &generatorList)
{
    return FSHelper::filterFilesByRegx(libSerachPaths, generatorList, GENERATOR_REGEX);
}

void CppLoader::getActuatorList(std::vector<std::string> &actuatorList)
{
    return FSHelper::filterFilesByRegx(libSerachPaths, actuatorList, ACTUATOR_REGEX);
}

