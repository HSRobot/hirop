/**
* @version 	v1.0.1
* @file		inspire_gripper.cpp
* @brief	因时夹抓相关动作插件
* @details	实现夹抓如下动作：
*               1、以设定的速度、力矩关闭夹抓
*               2、以设定的速度打开夹抓
*               3、急停
*               4、设置开口参数
*               v1.0.1变更内容：将串口打开程序段封装成服务
* @autor 	shencanjun
* @date		2019/11/5
*/

#include "serial_gripper.h"
SerialGripper::SerialGripper()
{

}

SerialGripper::~SerialGripper()
{

}

int SerialGripper::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node;
    if(!yamlNode["parameters"]){
        IErrorPrint("无参数设置");
        return -1;
    }
    node = yamlNode["parameters"];
    m_parm.serialNum = node["insprie"]["serialNum"].as<std::string>();
    m_parm.baudrate = node["insprie"]["baudrate"].as<uint32_t>();
    m_parm.speed = node["insprie"]["speed"].as<uint>();
    m_parm.force = node["insprie"]["force"].as<uint>();
    m_parm.size_max = node["insprie"]["size_max"].as<uint>();
    m_parm.size_min = node["insprie"]["size_min"].as<uint>();

#ifdef _COUT_
    std::cout<<"m_parm.serialNum:"<<m_parm.serialNum<<std::endl
              <<"m_parm.baudrate:"<<m_parm.baudrate<<std::endl
              <<"m_parm.speed:"<<m_parm.speed<<std::endl
              <<"m_parm.force:"<<m_parm.force<<std::endl
              <<"m_parm.size_max:"<<m_parm.size_max<<std::endl
              <<"m_parm.size_min:"<<m_parm.size_min<<std::endl;
#endif
    return 0;
}

int SerialGripper::connectGripper()
{
    try {
        ros_ser.setPort(m_parm.serialNum);
        ros_ser.setBaudrate(m_parm.baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    } catch (serial::IOException& e) {
        IErrorPrint("Unable to open port!!!");
        return -1;
    }
    IErrorPrint("Serial Port open ok ");
    return 0;
}

bool SerialGripper::isConnectGripper()
{
    bool state = false;
    try{
        state = ros_ser.isOpen();
    }catch(serial::IOException& e){
        IErrorPrint("Unable to open port!!!");
        return false;
    }
    return state;
}

int SerialGripper::disConnectGripper()
{
    try {
        ros_ser.close();
    } catch (serial::IOException& e) {
        IErrorPrint("Unable to open port!!!");
        return -1;
    }
    IDebug("Serial Port closed");
    return 0;
}

int SerialGripper::openGripper()
{
    if(m_parm.speed > 1000 || m_parm.speed <1)
    {
        IErrorPrint("The input speed is invalid. Please enter the number between 1-1000.");
        return -1;
    }

    unsigned char open_buffer[] = ARRAY_OPEN_GRIPPER;
    int size_open_buffer = ARRAY_SIZE(open_buffer);
L2:
    while( readGripperCurrenPose() <= openVal ){
        try {
            //将输入值req.speed保存为临时变量，以备后面数值处理
            int temp = m_parm.speed;
            //将低8位存入open_buffer[5]
            open_buffer[5]= (unsigned char)(temp&0xFF);
            //将高8位存入open_buffer[6]
            open_buffer[6]= (unsigned char)((temp&0xFF00)>>8);
            //close_buffer[7]为校验和位 （（B2+B3+…+B8）&0xFF）
            open_buffer[size_open_buffer-1]= (unsigned char)((open_buffer[2] + open_buffer[3] + \
                                                         open_buffer[4] + open_buffer[5] + \
                                                         open_buffer[6])&0xFF);
            ros_ser.write(open_buffer,size_open_buffer);
            IDebug("THE COMMAND OF --OPEN-- HAS BEEN LOADED!!!");
            usleep(100);
            ros_ser.flush();
            unsigned char read_buffer[9];
            int size_close_buffer = ARRAY_SIZE(read_buffer);
            unsigned char size = ros_ser.read(read_buffer,size_close_buffer );

        } catch (serial::IOException& e) {
            IErrorPrint("gripper open failed!!!");
            return -1;
        }
    }

    int temp = readGripperCurrenPose();
    if(temp <= openVal)
        goto L2;

    IErrorPrint("gripper open!");
    return 0;
}

int SerialGripper::closeGripper()
{
    if(m_parm.speed > 1000 || m_parm.speed <1)
    {
        IDebug("The input speed is invalid. Please enter the number between 1-1000.");
        return -1;
    }
    if(m_parm.force > 1000 || m_parm.force <50)
    {
        IDebug("The input force is invalid. Please enter the number between 50-1000.");
        return -1;
    }


    unsigned char close_buffer[] = ARRAY_CLOSE_GRIPPER;
    int size_close_buffer = ARRAY_SIZE(close_buffer);
L1:
    while(readGripperCurrenPose()  > closeVal ){
        try {
            //将输入值req.speed保存为临时变量，以备后面数值处理
            int temp = m_parm.speed;
            //将低8位存入close_buffer[5]
            close_buffer[5]= (unsigned char)(temp&0xFF);
            //将高8位存入close_buffer[6]
            close_buffer[6]= (unsigned char)((temp&0xFF00)>>8);

            //将输入值req.force保存为临时变量，以备后面数值处理
            temp = m_parm.force;
            //将低8位存入close_buffer[7]
            close_buffer[7]= (unsigned char)(temp&0xFF);
            //将高8位存入close_buffer[8]
            close_buffer[8]= (unsigned char)((temp&0xFF00)>>8);
            //close_buffer[9]为校验和位 （（B2+B3+…+B8）&0xFF）
            close_buffer[size_close_buffer-1]= (unsigned char)((close_buffer[2] + close_buffer[3] + \
                                                          close_buffer[4] + close_buffer[5] + \
                                                          close_buffer[6] + close_buffer[7] + close_buffer[8])&0xFF);
            ros_ser.write(close_buffer,size_close_buffer);
            usleep(100);
            ros_ser.flush();
            unsigned char read_buffer[9];
            int size_close_buffer = ARRAY_SIZE(read_buffer);
            unsigned char size = ros_ser.read(read_buffer,size_close_buffer );

            IDebug("THE COMMAND OF --CLOSE-- HAS BEEN LOADED!!!");
        } catch (serial::IOException& e) {
            IErrorPrint("gripper close failed!!!");
            return -1;
        }

    }
    int temp = readGripperCurrenPose();
    if(temp > 500)
        goto L1;

    IErrorPrint("gripper closed!");
    return 0;
}

int SerialGripper::stopGripper()
{
    unsigned char stop_buffer[] = ARRAY_STOP_GRIPPER;

    int size_stop_buffer = ARRAY_SIZE(stop_buffer);
    try {

        ros_ser.write(stop_buffer,size_stop_buffer);
        IDebug("THE COMMAND OF --STOP-- HAS BEEN LOADED!!!");

    } catch (serial::IOException& e) {
        IErrorPrint("gripper stop failed!");
        return -1;
    }
    IErrorPrint("gripper stop");
    return 0;
}



int SerialGripper::readGripperCurrenPose()
{
    //send
    unsigned char read_buffer_frame[] = ARRAY_READ_GRIPPER_POSE;
    int size_buffer = ARRAY_SIZE(read_buffer_frame);

    //read
    unsigned char read_buffer[8];
    size_buffer = ARRAY_SIZE(read_buffer);

    int val = -1;
    size_t s = 0;
    while(s == 0){
        ros_ser.flush();

        try {
            ros_ser.write(read_buffer_frame,size_buffer);
            usleep(100);
            s =ros_ser.read(read_buffer,size_buffer );
            val = bytes2int2(read_buffer);
        } catch (serial::PortNotOpenedException& e) {
            IErrorPrint("read size failed!");
        }
    }
//    IErrorPrint("CurrenPose is %d",val );
    return val;
}

int SerialGripper::setOpenSize()
{
    if(m_parm.size_max > 1000 || m_parm.size_max <0)
    {
        IDebug("The input req.max is invalid. Please enter the number between 0-1000.");
        return false;
    }
    if(m_parm.size_min > 1000 || m_parm.size_min <0)
    {
        IDebug("The input req.min is invalid. Please enter the number between 0-1000.");
        return false;
    }
    if(m_parm.size_max < m_parm.size_min || m_parm.size_max == m_parm.size_min)
    {
        IDebug("The input req.min and req.max are both invalid. Please ensure that req.min is less than req.max.");
        return false;
    }
    unsigned char buffer[] = ARRAY_SET_OPEN_SIZE_GRIPPER;

    int size_buffer = ARRAY_SIZE(buffer);

    try {
        //将输入值req.max保存为临时变量，以备后面数值处理
        int temp = m_parm.size_max;
        //将低8位存入close_buffer[5]
        buffer[5]= (unsigned char)(temp&0xFF);
        //将高8位存入close_buffer[6]
        buffer[6]= (unsigned char)((temp&0xFF00)>>8);

        //将输入值req.force保存为临时变量，以备后面数值处理
        temp = m_parm.size_min;
        //将低8位存入close_buffer[7]
        buffer[7]= (unsigned char)(temp&0xFF);
        //将高8位存入close_buffer[8]
        buffer[8]= (unsigned char)((temp&0xFF00)>>8);
        //close_buffer[9]为校验和位 （（B2+B3+…+B8）&0xFF）
        buffer[size_buffer-1]= (unsigned char)((buffer[2] + buffer[3] + \
                                                      buffer[4] + buffer[5] + \
                                                      buffer[6] + buffer[7] + buffer[8])&0xFF);

        ros_ser.write(buffer,size_buffer);
        IDebug("THE COMMAND OF --SET_OPEN_SIZE-- HAS BEEN LOADED!!!");
    } catch (serial::IOException& e) {
        IErrorPrint("set open size failed!");
        return -1;
    }

    IErrorPrint("set open size!");
    return 0;
}

unsigned char SerialGripper::getCheckNum(unsigned char* array, int lens)
{
    unsigned char sum = 0;
    for(int i = 2; i <= lens - 2 ; i++){
        sum +=array[i];
    }
    return sum&0XFF;
}

H_EXPORT_PLUGIN(SerialGripper,  "SerialGripper",  "1.1")
