#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <nav/mobile_robot.h>

using namespace hirop::navigation;

MobileRobot* MobileRobot::_instance =  NULL;

MobileRobot* MobileRobot::getInstance(){

    if(_instance == NULL)
        _instance = new MobileRobot();

    return _instance;
}

MobileRobot::MobileRobot(){

    _nodeHandle = new ros::NodeHandle();

    _odomSub = _nodeHandle->subscribe("/odom", 1, &MobileRobot::odomCallBack, this);

    _cmdvelPub = _nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    _motionThread = NULL;

}

int MobileRobot::stopMotion(){

    if(_motionThread != NULL){
        return 0;
    }

    _motion->stop();

}

int MobileRobot::runMotion(Motion *motion, bool block){

    if(_motionThread != NULL){
        return -1;
    }

    _motion = motion;

    if(block)
        __runMotion(motion);
    else{

        /**
         * 通过线程来启动motion
         */
        boost::function0<void> f =  boost::bind(&MobileRobot::__runMotion,this,  motion);

        _motionThread = new boost::thread(f);
        _motionThread->timed_join(boost::posix_time::microseconds(1));
    }

}

void MobileRobot::__runMotion(Motion *motion){

    /**
     *  执行初始化工作
     */
    motion->setup();

    /**
     * 根据Motion设置的频率来计算每次休眠的时间
     */
    int rate = motion->getRate();
    int duration = 0;

    if(rate > 0){
        duration = 1000 / rate;
    }

    /**
     *  周期性调用motion的update函数，直到update不再返回continue表明motion完成或者出现了错误。
     */
    while(motion->update() == Motion::CONTINUE){

        /**
         * 将motion生成的速度发布给底盘
         */
        updateCmdVel(motion->getCmdVel());

        if(rate == 0)
            break;

        boost::this_thread::sleep(boost::posix_time::milliseconds(duration));
    }

    std::cout << "Motion finished" << std::endl;

}

void MobileRobot::odomCallBack(const nav_msgs::OdometryConstPtr msgs){

    /**
     * 将ROS下的odom数据类型 转换为HIROP的odom类型
     */
    ROS_ODOM_2_HIROP_ODOM((*msgs), _odom);

}

void MobileRobot::updateCmdVel(hirop::Twist vel){

    geometry_msgs::Twist twist;

    HIROP_TWIST_2_ROS_TWIST(vel, twist);

    _cmdvelPub.publish(twist);
}

