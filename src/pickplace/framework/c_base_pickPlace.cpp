#include "pickplace/c_base_pickplace.h"

using namespace hirop_pickplace;

CBasePickPlace::CBasePickPlace()
{

}

int CBasePickPlace::getName(std::string &name)
{
    name = this->_name;
    return 0;
}

ENTITY_TYPE CBasePickPlace::getEntityType(){
    return this->_entityType;
}

int CBasePickPlace::parseConfig(YAML::Node &)
{
    return 0;
}

int CBasePickPlace::moveToFoName(std::string)
{
    return 0;
}

int CBasePickPlace::showObject(PoseStamped)
{
    return 0;
}

int CBasePickPlace::removeObject()
{
    return 0;
}
