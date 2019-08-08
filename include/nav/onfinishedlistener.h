#ifndef __ONFINISHEDLISTENER_H__
#define __ONFINISHEDLISTENER_H__

#include "motion.h"

namespace hirop{

namespace navigation{

class Motion;

/**
 * @brief Motion运动结束后的监听者接口类
 */
class OnFinishedListener{

public:
    virtual void OnFinished(Motion *motion);

};

}

}

#endif
