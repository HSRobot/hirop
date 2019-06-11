#include "vision/cpp_loader.h"

using namespace hirop_vision;

CppLoader::CppLoader(){
    this->loader = HPluginLoader::getLoader();
}

ITrainer *CppLoader::loadTrainer(std::string trainerName){

    HPlugin *plugin;
    loader->setPath("./");
    plugin = loader->load(trainerName);
    if(plugin == NULL)
        return NULL;

    return plugin->instance<ITrainer>();
}

IDetector *CppLoader::loadDetector(std::string detectorName){
    HPlugin *plugin;

    loader->setPath("./");
    plugin = loader->load(detectorName);
    if(plugin == NULL)
        return NULL;

    return plugin->instance<IDetector>();
}

