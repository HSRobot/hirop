#include "pickplace/configuer.h"

using namespace hirop_pickplace;

Configure::Configure(std::string fileName)
{
    try{
         config = YAML::LoadFile(fileName);
    }catch(std::exception e){
        IErrorPrint("Load configure file error");
    }
}

int Configure::getGeneratorName(std::string &generatorName){

    if(!config["generatorName"]){
        IErrorPrint("Get generator name error:  No generatorName node");
        return -1;
    }
    generatorName = config["generatorName"].as<std::string>();
    configDebug("Get generatorName: %s", generatorName.c_str());

    return 0;
}

int Configure::getPickPlacerName(std::string &pickpalcerName)
{
    if(!config["PickPlacerName"]){
        IErrorPrint("Get PickPlacer name error:  No PickPlacerName node");
        return -1;
    }
    pickpalcerName = config["PickPlacerName"].as<std::string>();
    configDebug("Get PickPlacerName: %s", pickpalcerName.c_str());

    return 0;
}

int Configure::getPrivateParams(YAML::Node &yamlNode)
{
    if(!config["parameters"]){
        IErrorPrint("无私有参数");
        return 0;
    }
    yamlNode = config;
    return 0;
}


