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
    commapi = new CommApi();
    proIO = new ProxyIO(commapi);
    commapi->setAutoConn(false);//关闭自动重连功能，否则连接失败
}

RobotIOGripper::~RobotIOGripper()
{
    proIO = NULL;
    commapi = NULL;
    delete proIO;
    delete commapi;
}

int RobotIOGripper::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node;
    if(!yamlNode["parameters"]){
        IErrorPrint("无参数设置");
        return -1;
    }
    node = yamlNode["parameters"];
    m_parm.robotIp = node["airpowere"]["robotIp"].as<std::string>();
    m_parm.robotPort = node["airpowere"]["robotPort"].as<ushort>();
    m_parm.portIndex = node["airpowere"]["portIndex"].as<int32_t>();
    m_parm.openVlaue = node["airpowere"]["openVlaue"].as<bool>();
    m_parm.closeValue = node["airpowere"]["closeValue"].as<bool>();

#ifdef _COUT_
    std::cout<<"m_parm.robotIp:"<<m_parm.robotIp<<std::endl
              <<"m_parm.robotPort:"<<m_parm.robotPort<<std::endl
              <<"m_parm.portIndex:"<<m_parm.portIndex<<std::endl
              <<"m_parm.openVlaue:"<<m_parm.openVlaue<<std::endl
              <<"m_parm.closeValue:"<<m_parm.closeValue<<std::endl;
#endif

    return 0;
}

int RobotIOGripper::connectGripper()
{
    ret = 1;
    if(commapi->isConnected()){
        IErrorPrint("already connect to hsc3!!!");
        return 0;
    }
    ret = commapi->connect(m_parm.robotIp, m_parm.robotPort);
    if(ret != 0){
        IErrorPrint("connect to hsc3 failed!!!,return val:" + ret);
        return -1;
    }
    return 0;
}

bool RobotIOGripper::isConnectGripper()
{
     return commapi->isConnected();
}

int RobotIOGripper::disConnectGripper()
{
    ret = 1;
    ret = commapi->disconnect();
    if(ret != 0){
        IErrorPrint("connect to hsc3 failed!!!,return val:" + ret);
        return -1;
    }
    return 0;
}

int RobotIOGripper::openGripper()
{
    ret = 1;

    ret = proIO->setDout(m_parm.portIndex,m_parm.openVlaue);
    if(ret != 0){
        IErrorPrint("set hsc3 IO failed!!!,return val:" + ret);
    }

    IErrorPrint("gripper open!");
    return 0;
}

int RobotIOGripper::closeGripper()
{
    ret = 1;
    ret = proIO->setDout(m_parm.portIndex,m_parm.closeValue);
    if(ret != 0){
        IErrorPrint("set hsc3 IO failed!!!,return val:" + ret);
    }
    IErrorPrint("gripper closed!");
    return 0;
}

H_EXPORT_PLUGIN(RobotIOGripper,  "RobotIOGripper",  "1.0")
