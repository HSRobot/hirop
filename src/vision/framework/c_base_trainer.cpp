#include "vision/c_base_trainer.h"
#include <utils/idebug.h>

using namespace hirop_vision;

CBaseTrainer::CBaseTrainer(std::string name){
    this->name =  name;
    this->entityType = CPP;
}

int CBaseTrainer::getName(std::string &name){
    name = this->name;
    return 0;
}

ENTITY_TYPE CBaseTrainer::getEntityType(){
    return this->entityType;
}

int CBaseTrainer::parseConfig(const YAML::Node &node){
    IDebug("using defult setConfig function");
}
