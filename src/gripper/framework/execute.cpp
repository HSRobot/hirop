#include "gripper/execute.h"

using namespace hirop_gripper;

Gripper::Gripper()
{
    cloaderPtr = new CppLoader();
    //pyLoader = PyLoader::getPyLoader();
}

Gripper::~Gripper()
{
    cloaderPtr = NULL;
    gripperPtr = NULL;
    configuerPtr = NULL;

    delete cloaderPtr;
    delete gripperPtr;
    delete configuerPtr;
}

int Gripper::setGripper(string gripperName, string entityType, std::string configFile)
{
    if(cppGripper.count(gripperName)){
        this->gripperPtr = cppGripper.at(gripperName);
    }
    else {
        this->gripperPtr = cloaderPtr->loadGripper(gripperName);
        if(this->gripperPtr == NULL){
            IErrorPrint("load Gripper: error!");
            return -1;
       }
       cppGripper.insert(std::pair<std::string, IGripper*>(gripperName, this->gripperPtr));
    }

    if( !configFile.empty()){
        IDebug("Gripper configFile: %s", configFile.c_str());
        YAML::Node privateParam;
        Configure *config = new Configure(configFile);
        if(!config->getPrivateParams(privateParam)){
            if(this->gripperPtr->parseConfig(privateParam) !=0 ){
                IErrorPrint("Get parseConfig  NULL");
                return -1;
            }
        }else{
            IErrorPrint("Get PrivateParam  NULL");
        }
    }
    return 0;
}

int Gripper::getGripperList(std::vector<std::string>& gripperList)
{
    this->cloaderPtr->getGripperList(gripperList);
    return 0;
}

int Gripper::connectGripper()
{
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL.");
        return -1;
    }

    if(this->gripperPtr->connectGripper() != 0){
        IErrorPrint("gripper connect is failed!!!");
        return -1;
    }
    IDebug("gripper connect is succeeful.");
    return 0;
}

bool Gripper::isConnectGripper()
{
    bool state = false;
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL.");
        return false;
    }

    state = this->gripperPtr->isConnectGripper();
    if(!state){
        IErrorPrint("gripper is  Not connect!!!");
        return state;
    }

    IDebug("gripper is connected");
    return state ;
}

int Gripper::disConnectGripper()
{
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL");
        return -1;
    }

    if(this->gripperPtr->disConnectGripper() != 0){
        IErrorPrint("gripper disconnect is  failed!!!");
        return -1;
    }
    IDebug("gripper disconnect is  succeeful!!!");
    return 0;
}

int Gripper::openGripper()
{
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL");
        return -1;
    }

    if(this->gripperPtr->openGripper() != 0){
        IErrorPrint("gripper open is  failed!!!");
        return -1;
    }
    IDebug("gripper open is  succeeful!!!");
    return 0;
}

int Gripper::closeGripper()
{
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL");
        return -1;
    }

    if(this->gripperPtr->closeGripper() != 0){
        IErrorPrint("gripper close is  failed!!!");
        return -1;
    }
    IDebug("gripper close is  succeeful!!!");
    return 0;
}


int Gripper::stopGripprt()
{
    if(this->gripperPtr == NULL){
        IErrorPrint("gripperPtr is  NULL");
        return -1;
    }
    if(this->gripperPtr->stopGripper() != 0){
        IErrorPrint("gripper stop is  failed!!!");
        return -1;
    }

    IDebug("gripper stop is  succeeful!!!");
    return 0;
}

