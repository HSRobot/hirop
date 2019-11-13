#ifndef C_BASE_GRIPPER_H
#define C_BASE_GRIPPER_H

#include "igripper.h"

namespace hirop_gripper {

class CBaseGripper:public IGripper{
public:
    CBaseGripper();

    int connectGripper();

    bool isConnectGripper();

    int disConnectGripper();

    int stopGripper();

    int getName(std::string &name);

    ENTITY_TYPE getEntityType();

    int parseConfig(YAML::Node&);

private:
    std::string _name;
    ENTITY_TYPE _entityType;
};
}
#endif // C_BASE_GRIPPER_H
