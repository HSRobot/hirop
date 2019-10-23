#ifndef C_BASE_PICKPLACE_H
#define C_BASE_PICKPLACE_H

#include "ipickplace.h"

namespace hirop_pickplace{

class CBasePickPlace:public IPickPlace{
public:

    CBasePickPlace();

    int getName(std::string &name) ;

    ENTITY_TYPE getEntityType() ;

    int parseConfig(YAML::Node&);

    int moveToFoName(std::string);

    int showObject(PoseStamped);

    int removeObject();

private:
    std::string _name;
    ENTITY_TYPE _entityType;
};
}
#endif // C_BASE_PICKPLACE_H
