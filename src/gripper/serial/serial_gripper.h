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

#include <serial/serial.h>

#include "gripper/c_base_gripper.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/* USB转串口设备号 */
#define SERIAL_DEV "/dev/ttyUSB0"
/* 串口波特率 */
#define SERIAL_BAUDRATE 115200

/* 夹抓相关动作的默认串口输入值 */
#define ARRAY_CLOSE_GRIPPER {0xEB,0x90,0x01,0x05,0x10,0xF4,0x01,0x64,0x00,0x6F}
#define ARRAY_OPEN_GRIPPER {0xEB,0x90,0x01,0x03,0x11,0xF4,0x01,0x0A}
#define ARRAY_STOP_GRIPPER {0xEB,0x90,0x01,0x01,0x16,0x18}
#define ARRAY_SET_OPEN_SIZE_GRIPPER {0xEB,0x90,0x01,0x05,0x12,0xE8,0x03,0x70,0x00,0x73}
#define ARRAY_GET_OPEN_SIZE_GRIPPER {0xEB,0x90,0x01,0x01,0x13,0x15}
#define ARRAY_READ_GRIPPER_POSE {0xEB,0x90,0x01,0x01,0xD9}


using namespace hirop_gripper;

class SerialGripper:public CBaseGripper{

typedef struct Parameters{
        std::string serialNum;
        uint32_t baudrate;
        uint speed;
        uint force;
        uint size_max;
        uint size_min;
    }Parameters;

public:
    SerialGripper();
    ~SerialGripper();

    int parseConfig(YAML::Node& );

    int connectGripper();

    bool isConnectGripper();

    int disConnectGripper();

    int openGripper();

    int closeGripper();

    int stopGripper();
    int readGripperCurrenPose();

private:
    int setOpenSize();

private:
    serial::Serial ros_ser;
    Parameters m_parm;
};

H_DECLARE_PLUGIN(IGripper)
