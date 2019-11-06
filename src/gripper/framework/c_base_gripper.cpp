#include "gripper/c_base_gripper.h"

using namespace hirop_gripper;

CBaseGripper::CBaseGripper()
{

}

int CBaseGripper::connectGripper()
{
    return 0;
}

bool CBaseGripper::isConnectGripper()
{
    return true;
}

int CBaseGripper::disConnectGripper()
{
    return 0;
}

int CBaseGripper::stopGripper()
{
    return 0;
}

int CBaseGripper::getName(std::string &name)
{
    name = this ->_name;
    return 0;
}

ENTITY_TYPE CBaseGripper::getEntityType()
{
    return this->_entityType;
}

int CBaseGripper::parseConfig(YAML::Node &)
{
    return 0;
}
