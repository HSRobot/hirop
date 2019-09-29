#ifndef __MOTION_H__
#define __MOTION_H__

#include <iostream>
#include <vector>

#include "../msgs/twist.h"

#include "onfinishedlistener.h"

namespace hirop{

namespace navigation{

enum MotionResult {FINISHED, ERROR, CONTINUE};

class Motion{

public:

    /**
     * @brief Motion    构造函数
     */
    Motion(std::string name, int rate): _name(name), _rate(rate){}

    /**
     * @brief ~Motion    析构函数
     */
    virtual ~Motion();

    /**
     * @brief setup     在Motion运动之前会调用该函数来进行相关的初始化
     * @return          0 表示系统初始化成功 -1 表示初始化失败
     */
    virtual int setup() = 0;

    /**
     * @brief update    每个周期会调用该函数来更新要发送给底盘的最终速度
     * @return
     */
    virtual MotionResult update() = 0;

    /**
     * @brief stop      停止当前的motion
     */
    virtual int stop() = 0;

    /**
     * @brief getRate   获取更新的频率
     * @return          更新频率
     */
    int getRate() const {return _rate;}

    /**
     * @brief setOnFinishListener   设置运动结束事件的监听者
     * @return                      0 成功 -1 失败
     */
    int setOnFinishedListener(OnFinishedListener *lister);

    /**
     * @brief getVelCmd         获取当前周期的cmdvel
     * @return                  cmdvel
     */
    hirop::Twist getCmdVel() const {return _cmdvel;}

protected:
    /**
     * @brief _cmdvel   保存当前周期的底盘速度控制结果
     */
    hirop::Twist _cmdvel;

private:
    /**
     * @brief _rate      速度更新的频率，如果为0表示，算法不需要更新
     */
    int _rate;

    /**
     * @brief _name    保存当前motion的名称
     */
    std::string _name;

    /**
     * @brief _listers  保存监听者的列表
     * @todo 需要改成智能指针
     */
    std::vector<OnFinishedListener *> _listers;

};

}

}

#endif
