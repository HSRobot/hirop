#pragma once

#include "motion.h"

namespace hirop{

namespace navigation{

class CircleMotion : public Motion{

public:
    /**
     * @brief CircleMotion  构造函数
     * @param r             圆周运动的半径，如果为0则表示原地旋转
     * @param vel           圆周运动的线速度
     * @param rate          更新频率
     */
    CircleMotion(float r, float vel, int rate=40) : Motion("CircleMotion", rate) , _radius(r), _linearVel(vel){}

    /**
     *  析构函数
     */
    ~CircleMotion();

    /**
     * @brief setup 运动的初始化工作在这个函数中进行
     */
    int setup();

    /**
     * @brief stop
     * @return
     */
    int stop();

    /**
     * @brief update    每个周期调用一次，Motion在里面实现当前周期速度的计算
     * @return          Motion的结果
     */
    RETURN_TYPE update();

private:
    /**
     * @brief _radius    运动的半径
     */
    float _radius;

    /**
     * @brief _linearVel 直线速度
     */
    float _linearVel;

    /**
     * @brief _angleVel 角速度
     */
    float _angleVel;
};

}

}
