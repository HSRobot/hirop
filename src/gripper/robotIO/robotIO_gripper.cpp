/**
* @version 	v1.0.1
* @file		inspire_gripper.cpp
* @brief	气动夹爪相关动作插件
* @details	实现夹爪如下动作：
*               1、设置华数三型的IO输出控制气动夹爪
*               2、控制爪的开合
* @autor 	shencanjun
* @date		2019/11/６
*/

#include "robotIO_gripper.h"

RobotIOGripper::RobotIOGripper()
{
    n_gripper = ros::NodeHandle();
    hsc3Client = n_gripper.serviceClient<hirop_msgs::setIODout>("hsc3SetIODout");
}

RobotIOGripper::~RobotIOGripper()
{

}

int RobotIOGripper::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node;
    if(!yamlNode["parameters"]){
        IErrorPrint("无参数设置");
        return -1;
    }
    node = yamlNode["parameters"];
    m_parm.portIndex = node["airpowere"]["portIndex"].as<int32_t>();
    m_parm.openVlaue = node["airpowere"]["openVlaue"].as<bool>();
    m_parm.closeValue = node["airpowere"]["closeValue"].as<bool>();

#ifdef _COUT_
    std::cout<<"m_parm.portIndex:"<<m_parm.portIndex<<std::endl
              <<"m_parm.openVlaue:"<<m_parm.openVlaue<<std::endl
              <<"m_parm.closeValue:"<<m_parm.closeValue<<std::endl;
#endif

    return 0;
}

int RobotIOGripper::openGripper()
{
    try {
        setIo_srv.request.value = m_parm.openVlaue;
        setIo_srv.request.portIndex = m_parm.portIndex;

        if(!(hsc3Client.call(setIo_srv))){
            IErrorPrint("gripper server call failed!!!");
            return -1;
        }

    } catch (ros::Exception& e) {
        IErrorPrint("gripper server call error!!!");
        return -1;
    }
    IErrorPrint("gripper open!");
    return 0;
}

int RobotIOGripper::closeGripper()
{
    try {
        setIo_srv.request.value = m_parm.closeValue;
        setIo_srv.request.portIndex = m_parm.portIndex;

        if(!(hsc3Client.call(setIo_srv))){
            IErrorPrint("gripper server call failed!!!");
            return -1;
        }

    } catch (ros::Exception& e) {
        IErrorPrint("gripper server call error!!!");
        return -1;
    }
    IErrorPrint("gripper closed!");
    return 0;
}

H_EXPORT_PLUGIN(RobotIOGripper,  "RobotIOGripper",  "1.0")
