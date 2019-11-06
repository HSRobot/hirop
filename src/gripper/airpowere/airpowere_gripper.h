/** 
* @version 	v1.0.1 
* @file		gripper_close_srv.cpp
* @brief	夹抓相关动作的服务端
* @details	实现夹抓如下动作：
*               1、以设定的速度、力矩关闭夹抓
*               2、以设定的速度打开夹抓
*               3、急停
*               4、设置开口参数
*               v1.0.1变更内容：将串口打开程序段封装成服务
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#include <ros/ros.h>

#include "hirop_msgs/setIODout.h"

#include "gripper/c_base_gripper.h"

using namespace hirop_gripper;

class AirpowereGripper:public CBaseGripper{

typedef struct Parameters{
        int32_t portIndex;
        bool openVlaue;
        bool closeValue;
    }Parameters;

public:
    AirpowereGripper();
    ~AirpowereGripper();

    int parseConfig(YAML::Node& );

    int openGripper();

    int closeGripper();

private:
    Parameters m_parm;
    ros::NodeHandle n_gripper;
    ros::ServiceClient hsc3Client;
    hirop_msgs::setIODout setIo_srv;

};

H_DECLARE_PLUGIN(IGripper)
