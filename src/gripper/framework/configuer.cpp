#include "gripper/configuer.h"

using namespace hirop_gripper;

Configure::Configure(std::string fileName)
{
    try{
         config = YAML::LoadFile(fileName);
    }catch(std::exception e){
        IErrorPrint("Load configure file error");
    }
}

int Configure::getGripperName(std::string &gripperName){

    if(!config["gripperName"]){
        IErrorPrint("Get gripper name error:  No gripperName node");
        return -1;
    }
    gripperName = config["gripperName"].as<std::string>();
    configDebug("Get gripperName: %s", gripperName.c_str());

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


