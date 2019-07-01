#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "vision/cpp_loader.h"


using namespace hirop_vision;

CppLoader::CppLoader(){
    this->loader = HPluginLoader::getLoader();

    libSerachPath = getenv(LIB_SERACH_PATH_ENV);
    std::vector<std::string> paths;
    boost::split(paths, libSerachPath, boost::is_any_of(":"));

    /**
     * @brief 从环境变量中获取库的搜索路径，支持多路径
     */
    BOOST_FOREACH(std::string path, paths){
        libSerachPaths.push_back(path + "/");
    }

}

ITrainer *CppLoader::loadTrainer(std::string trainerName){

    HPlugin *plugin;

    /**
     * @brief 尝试在所有路径中寻找相关的库
     */
    BOOST_FOREACH(std::string path, libSerachPaths){
        loader->setPath(path);
        plugin = loader->load(trainerName + "Trainer");
        if(plugin != NULL)
            break;
    }

    if(plugin == NULL)
        return NULL;

    return plugin->instance<ITrainer>();
}

IDetector *CppLoader::loadDetector(std::string detectorName){
    HPlugin *plugin;

    /**
     * @brief 尝试在所有路径中寻找相关的库
     */
    BOOST_FOREACH(std::string path, libSerachPaths){
        loader->setPath(path);
        plugin = loader->load(detectorName + "Detector");
        if(plugin != NULL)
            break;
    }

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
            if(boost::xpressive::regex_search(pos->path().filename().string().c_str(), what, reg)){
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

            /**
         * 通过正则表达式 获取文件名中的检测器名称
         */
            if(boost::xpressive::regex_search(pos->path().filename().string().c_str(), what, reg)){
                /**
             * 这里我们假定匹配到的第一个字符串就是识别器的名称
             */
                trainerList.push_back(what[0]);
            }
        }
    }
}
