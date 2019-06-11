#include "vision/configure.h"

using namespace hirop_vision;

Configure::Configure(std::string fileName){
    try{
    config = YAML::LoadFile(fileName);
    }catch(std::exception e){
        std::cerr << "Load configure file error" << std::endl;
    }
}

int Configure::getObjectName(std::string &objName){
    if(!config["objectName"]){
        std::cerr << "Get Object name error:  No objectName node" << std::endl;
        return -1;
    }
    objName = config["objectName"].as<std::string>();
    configDebug("Get objectName: %s", objName.c_str());

    return 0;
}

int Configure::getTrainerName(std::string &trainerName){

    if(!config["trainerName"]){
        std::cerr << "Get tariner name error:  No tarinerName node" << std::endl;
        return -1;
    }
    trainerName = config["trainerName"].as<std::string>();
    configDebug("Get tarinerName: %s", trainerName.c_str());

    return 0;
}
