#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "vision/cpp_loader.h"

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
    libSerachPaths.push_back("");

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

    /**
     * @brief reg   正则表达式
     */
    boost::xpressive::cregex reg = boost::xpressive::cregex::compile(DETECTOR_REGEX);

    BOOST_FOREACH(std::string path, libSerachPaths){

        boost::filesystem::path libDir(path);
        boost::filesystem::directory_iterator end;

        /**
     * 遍历目录，获取目录下的所有文件
     */
        for(boost::filesystem::directory_iterator pos(libDir); pos != end; ++pos){
            if(boost::filesystem::is_directory(*pos)){
                continue;
            }

            /**
         * @brief what  临时保存正则表达式的查找结果
         */
            boost::xpressive::cmatch what;

            /**
         * 通过正则表达式 获取文件名中的检测器名称
         */
            std::string tmp(pos->path().filename().string());

            /**
             *  如果直接使用pos->path().filename().string().c_str()作为第一参数的话，那么是可悲的。
             *  因为还有调用regex_search这个函数的时候，这个c_str所指向的char *里的数据就已经没了。
             *  因为c_str返回的数据是由string维护的
             */
            if(boost::xpressive::regex_search(tmp.c_str(), what, reg)){
                /**
             * 这里我们假定匹配到的第一个字符串就是识别器的名称
             */
                detectorList.push_back(what[0]);
            }
        }
    }
}

void CppLoader::getTrainerList(std::vector<std::string> &trainerList){
    /**
     * @brief reg   正则表达式
     */
    boost::xpressive::cregex reg = boost::xpressive::cregex::compile(TRAINER_REGEX);

    BOOST_FOREACH(std::string path, libSerachPaths){

        boost::filesystem::path libDir(path);
        boost::filesystem::directory_iterator end;

        /**
     * 遍历目录，获取目录下的所有文件
     */
        for(boost::filesystem::directory_iterator pos(libDir); pos != end; ++pos){
            if(boost::filesystem::is_directory(*pos)){
                continue;
            }

            /**
         * @brief what  临时保存正则表达式的查找结果
         */
            boost::xpressive::cmatch what;

            std::string tmp(pos->path().filename().string());

            /**
         * 通过正则表达式 获取文件名中的检测器名称
         */
            if(boost::xpressive::regex_search(tmp.c_str(), what, reg)){
                /**
             * 这里我们假定匹配到的第一个字符串就是识别器的名称
             */
                trainerList.push_back(what[0]);
            }
        }
    }
}
