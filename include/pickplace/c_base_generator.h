#ifndef C_BASE_GENERATOR_H
#define C_BASE_GENERATOR_H

#include "igenerator.h"

namespace hirop_pickplace {

class CBaseGenerator:public IGenerator{
public:
    CBaseGenerator();

    int getName(std::string &name);

    ENTITY_TYPE getEntityType();

    int parseConfig(YAML::Node&);

private:
    std::string _name;
    ENTITY_TYPE _entityType;
};
}
#endif // C_BASE_GENERATOR_H
