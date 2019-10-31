#include "pickplace/execute_process.h"

using namespace hirop_pickplace;

PickPlace::PickPlace()
{
    cloaderPtr = new CppLoader();
    //pyLoader = PyLoader::getPyLoader();
    stopFlag_ = false;
}

PickPlace::~PickPlace()
{
    cloaderPtr = NULL;
    generatorPtr = NULL;
    pickplacePtr = NULL;
    configuerPtr = NULL;

    delete cloaderPtr;
    delete generatorPtr;
    delete pickplacePtr;
    delete configuerPtr;
}

int PickPlace::setGenerator(string generatorName, string entityType, std::string configFile)
{
    if(cppGenerator.count(generatorName)){
        this->generatorPtr = cppGenerator.at(generatorName);
    }
    else {
        this->generatorPtr = cloaderPtr->loadGenerator(generatorName);

        if(this->generatorPtr == NULL){
            IErrorPrint("load Generator: error!");
            return -1;
       }
       cppGenerator.insert(std::pair<std::string, IGenerator*>(generatorName, this->generatorPtr));
    }

    if( !configFile.empty()){
        IDebug("Genertor configFile: %s\n",configFile);
        YAML::Node privateParam;
        Configure *config = new Configure(configFile);
        if(!config->getPrivateParams(privateParam)){
            if(this->generatorPtr->parseConfig(privateParam) !=0 ){
                IErrorPrint("Get parseConfig  NULL");
                return -1;
            }
        }else{
            IErrorPrint("Get PrivateParam  NULL");
        }
    }
    return 0;
}

int PickPlace::setActuator(string actuatorName, string entityType, std::string configFile)
{
    if(cppActuator.count(actuatorName)){
        this->pickplacePtr = cppActuator.at(actuatorName);
    }
    else {
        this->pickplacePtr = cloaderPtr->loadPickPlace(actuatorName);

        if(this->pickplacePtr == NULL){
            IErrorPrint("load Actuator: error!");
            return -1;
        }
        cppActuator.insert(std::pair<std::string, IPickPlace*>(actuatorName, this->pickplacePtr));
    }

    if( !configFile.empty()){
        IDebug( "actuator configFile: %s\n", configFile);
        YAML::Node privateParam;
        Configure *config = new Configure(configFile);
        if(!config->getPrivateParams(privateParam)){
            if(this->pickplacePtr->parseConfig(privateParam) !=0 ){
                IErrorPrint("Get parseConfig  NULL");
                return -1;
            }
        }else{
            IErrorPrint("Get PrivateParam  NULL");
        }
    }

    return 0;
}

int PickPlace::getGeneratorList(std::vector<std::string>& generatorList)
{
    this->cloaderPtr->getGeneratorList(generatorList);
    return 0;
}

int PickPlace::getActuatorList(std::vector<std::string> &actuatorList)
{
    this->cloaderPtr->getActuatorList(actuatorList);
    return 0;
}

int PickPlace::setPickPose(PoseStamped pickPos)
{
    if(this->generatorPtr == NULL){
        return -1;
    }

    objPos_ = pickPos;

    this->generatorPtr->setPickPose(pickPos);
    if(this->generatorPtr->genPickPose() != 0){
        return -1;
    }
    return 0;
}

int PickPlace::setPlacePose(PoseStamped placePos)
{
    if(this->generatorPtr == NULL){
        return -1;
    }
    this->generatorPtr->setPlacePose(placePos);
    if(this->generatorPtr->genPlacePose() != 0){
        return -1;
    }
    return 0;
}

int PickPlace::showObject(PoseStamped objPose)
{
    if(this->pickplacePtr == NULL)
        return -1;
    if(objPose.pose.position.x == 0 && objPose.pose.position.y == 0 && objPose.pose.position.z == 0){
        objPose = objPos_;
    }
    if(this->pickplacePtr->showObject(objPose))
        return -1;
    return 0;
}

int PickPlace::removeObject()
{
    if(this->pickplacePtr == NULL)
        return -1;
    if(this->pickplacePtr->removeObject())
        return -1;
    return 0;
}

int PickPlace::moveToPos(PoseStamped pose)
{
    if(this->pickplacePtr == NULL){
        IErrorPrint("pickplacePtr: Null!!!")
        return -1;
    }
    if(this->pickplacePtr->moveToPos(pose) == 0){
        IDebug("moveToPos succeeful! ! !");
    }
    else{
        IErrorPrint("moveToPos failed! ! !");
        return -1;
    }
    return 0;
}

int PickPlace::moveToFoName(string poseName)
{
    if(this->pickplacePtr == NULL){
        IErrorPrint("pickplacePtr: Null!!!");
    }
    if(this->pickplacePtr->moveToFoName(poseName) == 0){
        IDebug("moveToFoName succeeful! ! !");
    }
    else {
        IErrorPrint("moveToFoName failed! ! !");
        return -1;
    }
    return 0;
}

int PickPlace::pick()
{
    stopFlag_ = false;
    PoseStamped pickPoss;
    removeObject();
    if(this->generatorPtr == NULL){
        IErrorPrint("generatorPtr: Null! ! !");
        return -1;
    }
    if(this->pickplacePtr== NULL){
        IErrorPrint("pickplacePtr: Null! ! !");
        return -1;
    }

    this->generatorPtr->getResultPickPose(pickPoss);
    showObject(objPos_);

    if(stopFlag_)
        return -1;

    if(this->_pick(pickPoss)  == 0 && ! stopFlag_){
        IDebug("pick succeeful! ! !");
    }
    else{
        IErrorPrint("pick failed! ! !");
        return -1;
    }

    return 0;
}

int PickPlace::place()
{
    stopFlag_ = false;
    PoseStamped placePoss;

    if(this->generatorPtr == NULL){
        IErrorPrint("generatorPtr: Null!!!");
        return -1;
    }
    if(this->pickplacePtr == NULL){
        IErrorPrint("pickplacePtr: Null!!!");
        return -1;
    }

    this->generatorPtr->getResultPlacePose(placePoss);

    if(this->_place(placePoss)  == 0 && ! stopFlag_){
        IDebug("place succeeful! ! !");
    }

    else{
        IErrorPrint("place failed! ! !");
        return -1;
    }
    removeObject();
    return 0;
}

int PickPlace::_pick(PoseStamped pickPos)
{
     this->pickplacePtr->setPickPose(pickPos);
     return this->pickplacePtr->pick();
}

int PickPlace::_place(PoseStamped placePos)
{
    this->pickplacePtr->setPlacePose(placePos);
    return this->pickplacePtr->place();
}

int PickPlace::stop()
{
    stopFlag_ = true;
    generatorPtr->stopGenerator();
    pickplacePtr->stopPickplace();
    return 0;
}

