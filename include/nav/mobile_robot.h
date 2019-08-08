#ifndef __MOBILE_ROEBOT_H__
#define __MOBILE_ROEBOT_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>

#include <msgs/odometry.h>

#include "motion.h"

namespace hirop{

namespace navigation{

class MobileRobot{

public:
    /**
     * @brief getInstance   获取MobileRobot的唯一实例
     * @return              MobileRobot实例
     */
    static MobileRobot *getInstance();

    /**
     * @brief stopMotion    停止当前运动
     * @return
     */
    int stopMotion();

    /**
     * @brief runMotion     执行运动
     * @return
     */
    int runMotion(Motion *motion);

    /**
     * @brief setListener   设置监听者
     * @return
     */
    //int setListener();

    /**
     * @brief getOdom       获取最新的里程计信息
     * @return              最新的里程计信息
     */
    hirop::Odometry getOdom() const {return _odom;}


private:
    /**
     * @brief MobileRobot   构造函数
     */
    MobileRobot();

    /**
     * @brief __runMotion   线程函数，将会以一个线程的形式进行执行
     * @param motion        被执行的motion
     */
    void __runMotion(Motion *motion);

    /**
     * @brief updateVel     更新底盘的速度
     * @param vel           速度
     */
    void updateCmdVel(hirop::Twist vel);

    /**
     * @brief odomCallBack  ROS odom话题的监听器
     * @param msgs          最新的odom数据
     */
    void odomCallBack(const nav_msgs::OdometryConstPtr msgs);

private:

    /**
     * @brief _instance     单例的是实例
     */
    static MobileRobot *_instance;

    /**
     * @brief _odom         保存着最新的odom
     */
    hirop::Odometry _odom;

    /**
     * @brief _nodeHandle   保存了ROS的节点句柄
     */
    ros::NodeHandle *_nodeHandle;

    /**
     * @brief _cmdvelPub    cmdvel的发布者
     */
    ros::Publisher _cmdvelPub;

    /**
     * @brief _odomSub      odom的订阅者
     */
    ros::Subscriber _odomSub;

    /**
     * @brief _motionThread 运动线程
     */
    boost::thread *_motionThread;

    /**
     * @brief motion    保存当前正在运行的Motion
     */
    Motion *_motion;

};

}

}


#endif
