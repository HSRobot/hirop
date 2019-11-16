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

#include "gripper/c_base_gripper.h"

#include "hsc3/CommApi.h"
#include "hsc3/proxy/ProxyIO.h"
#include "exception"

using namespace hirop_gripper;
using namespace Hsc3::Comm;
using namespace Hsc3::Proxy;

class RobotIOGripper:public CBaseGripper{

typedef struct Parameters{
        std::string robotIp;
        ushort robotPort;
        int32_t portIndex;
        bool openVlaue;
        bool closeValue;
    }Parameters;

public:
    RobotIOGripper();
    ~RobotIOGripper();

    int parseConfig(YAML::Node& );

    int connectGripper();

    bool isConnectGripper();

    int disConnectGripper();

    int openGripper();

    int closeGripper();

private:
    CommApi *commapi;
    ProxyIO *proIO;
    HMCErrCode ret;

    Parameters m_parm;
};

H_DECLARE_PLUGIN(IGripper)
