#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "vision/cpp_loader.h"

#include "utils/fs_helper.h"

using namespace hirop_vision;

CppLoader::CppLoader(){

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

ITrainer *CppLoader::loadTrainer(std::string trainerName){

    HPlugin *plugin;

    loader->setPaths(libSerachPaths);
    plugin = loader->load(trainerName + "Trainer");

    if(plugin == NULL)
        return NULL;

    return plugin->instance<ITrainer>();
}

IDetector *CppLoader::loadDetector(std::string detectorName){
    HPlugin *plugin;

    loader->setPaths(libSerachPaths);
    plugin = loader->load(detectorName + "Detector");

    if(plugin == NULL)
        return NULL;

    return plugin->instance<IDetector>();
}


void CppLoader::getDetectorList(std::vector<std::string> &detectorList){
    return FSHelper::filterFilesByRegx(libSerachPaths, detectorList, DETECTOR_REGEX);
}

void CppLoader::getTrainerList(std::vector<std::string> &trainerList){
    return FSHelper::filterFilesByRegx(libSerachPaths, trainerList, TRAINER_REGEX);
}
