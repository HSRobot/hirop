#include <force/forcePluginAggre.h>
#include <boost/make_shared.hpp>
#include <force/forceCppLoader.h>
#include <force/fileConfiguer.h>
#include <force/forceExport.h>
using namespace hirop_force;

forcePluginAggre::forcePluginAggre()
{
    forceCppLoadPtr = boost::make_shared<forceCppLoader>();
}

forcePluginAggre::~forcePluginAggre()
{

}

int forcePluginAggre::setForcePlugin(string generatorName, string entityType, std::string configFile)
{
    if(cppforcePtrMap.count(generatorName)){
        forcePtr = cppforcePtrMap.at(generatorName);
    }
    else {
        IForce* ptr = forceCppLoadPtr->loadForcePlugin(generatorName);
        forcePtr = boost::shared_ptr<IForce>(ptr);
        if(forcePtr == NULL){
            IErrorPrint("load Force: error!");
            return -1;
       }
       cppforcePtrMap.insert(std::pair<std::string, boost::shared_ptr<IForce>>(generatorName, forcePtr));
    }

    if( !configFile.empty()){
        IErrorPrint("Force configFile: %s", configFile.c_str());
        YAML::Node privateParam;
        boost::shared_ptr<fileConfiguer> fileConfiguerHandler = boost::make_shared<fileConfiguer>(configFile);
        if(!fileConfiguerHandler->getPrivateParams(privateParam)){
            //
            if(forcePtr->parseConfig(privateParam) !=0 ){
                IErrorPrint("Get parseConfig  NULL");
                return -1;
            }
            IErrorPrint("Get parseConfig  Sucessfully");
        }else{
            IErrorPrint("Get PrivateParam  NULL");
        }
    }
    return 0;
}

int forcePluginAggre::getForcePluginList(std::vector<std::string> &forceList)
{
    this->forceCppLoadPtr->getForcePluginList(forceList);
    return 0;
}

int forcePluginAggre::parseConfig(YAML::Node &node)
{
    forcePtr->parseConfig(node);
}

void forcePluginAggre::setInputForceBias(std::vector<double> &pose)
{
    forcePtr->setInputForceBias(pose);
}

void forcePluginAggre::setInputRobotPose(std::vector<double> &pose)
{
    forcePtr->setInputRobotCartPose(pose);
}

int forcePluginAggre::compute()
{
    return forcePtr->compute();
}

int forcePluginAggre::getResult(std::vector<double> &pose)
{
    return forcePtr->getResult(pose);
}

string forcePluginAggre::getName()
{
    return forcePtr->getName();
}



int forcePluginAggre::printInfo()
{
    forcePtr->printInfo();
}



