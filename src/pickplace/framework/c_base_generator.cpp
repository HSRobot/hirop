#include "pickplace/c_base_generator.h"

using namespace hirop_pickplace;

CBaseGenerator::CBaseGenerator()
{

}

int CBaseGenerator::getName(std::string &name)
{
    name = this ->_name;
    return 0;
}

ENTITY_TYPE CBaseGenerator::getEntityType()
{
    return this->_entityType;
}

int CBaseGenerator::parseConfig(YAML::Node &)
{
    return 0;
}
