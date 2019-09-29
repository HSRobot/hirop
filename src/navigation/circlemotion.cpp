#include <nav/circlemotion.h>

using namespace hirop::navigation;

CircleMotion::~CircleMotion(){

}

int CircleMotion::setup(){

    /**
     *  计算角速度
     */
    _angleVel = _linearVel / _radius;

    _cmdvel.angular.x = 0;
    _cmdvel.angular.y = 0;
    _cmdvel.angular.z = 0;

    _cmdvel.linear.x = 0;
    _cmdvel.linear.y = 0;
    _cmdvel.linear.z = 0;

}

int CircleMotion::stop(){

    return 0;
}

MotionResult CircleMotion::update(){

    _cmdvel.angular.z = _angleVel;
    _cmdvel.linear.x = _linearVel;

    return CONTINUE;
}
