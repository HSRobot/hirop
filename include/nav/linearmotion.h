#ifndef __LINEARMOTION_H__
#define __LINEARMOTION_H__

#include <ros/ros.h>

#include "motion.h"

namespace hirop{

namespace navigation {

class LinearMotion : public Motion{

public:

    /**
     * @brief LinearMotion  构造函数
     * @param distance      直线运动距离
     * @param vel           速度
     * @param rate          更新频率
     */
    LinearMotion(float distance, float vel, int rate = 50) : Motion("LinearMotion", rate),\
        _vel(vel), _distance(distance){}

    /**
     *  析构函数
     */
    ~LinearMotion();

    /**
     * @brief setup     初始化
     * @return          0 成功 -1 失败
     */
    int setup();

    /**
     * @brief stop      停止运动
     * @return          0 成功 -1 失败
     */
    int stop();

    /**
     * @brief update    周期更新函数
     * @return          更新结果
     */
    RETURN_TYPE update();

private:
    /**
     * @brief _distance 前进的目标距离
     */
    float _distance;

    /**
     * @brief _vel      运动时的速度
     */
    float _vel;

    /**
     * @brief test      测试用
     */
    int test;
};

}

}

#endif
