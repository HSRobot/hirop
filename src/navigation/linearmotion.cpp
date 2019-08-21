#include <nav/linearmotion.h>

using namespace hirop::navigation;

int LinearMotion::setup(){

    /**
     *  在这里获取初始的里程计信息
     */
    std::cout << "in linear motion setup" << std::endl;

    test = 0;

    _cmdvel.angular.x = 0;
    _cmdvel.angular.y = 0;
    _cmdvel.angular.z = 0;

    _cmdvel.linear.x = 0;
    _cmdvel.linear.y = 0;
    _cmdvel.linear.z = 0;

    return 0;
}

int LinearMotion::stop(){
    return 0;
}

Motion::RETURN_TYPE LinearMotion::update(){

    if(test > 50){

        _cmdvel.linear.x = 0;
        return FINISHED;

    }

    test ++;

    _cmdvel.linear.x = 0.5;

    return CONTINUE;
}


LinearMotion::~LinearMotion(){

}
