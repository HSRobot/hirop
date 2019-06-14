#include <boost/xpressive/xpressive_dynamic.hpp>

#include "vision/cpp_loader.h"

using namespace hirop_vision;

CppLoader::CppLoader(){
    this->loader = HPluginLoader::getLoader();
    libSerachPath = LIB_SERACH_PATH;
}

ITrainer *CppLoader::loadTrainer(std::string trainerName){

    HPlugin *plugin;
    loader->setPath(libSerachPath);
    plugin = loader->load(trainerName + "Trainer");
    if(plugin == NULL)
        return NULL;

    return plugin->instance<ITrainer>();
}

IDetector *CppLoader::loadDetector(std::string detectorName){
    HPlugin *plugin;

    loader->setPath(libSerachPath);
    plugin = loader->load(detectorName+"Detector");
    if(plugin == NULL)
        return NULL;

    return plugin->instance<IDetector>();
}


void CppLoader::getDetectorList(std::vector<std::string> &detectorList){

    /**
     * @brief reg   正则表达式
     */
    boost::xpressive::cregex reg = boost::xpressive::cregex::compile(DETECTOR_REGEX);

    boost::filesystem::path libDir(libSerachPath);
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

void CppLoader::getTrainerList(std::vector<std::string> &trainerList){
    /**
     * @brief reg   正则表达式
     */
    boost::xpressive::cregex reg = boost::xpressive::cregex::compile(TRAINER_REGEX);

    boost::filesystem::path libDir(libSerachPath);
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
