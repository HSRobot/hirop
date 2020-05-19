#pragma once

#include "serial_gripper.h"
/**
   相关的发送字符串
 **/
//最后一位是校验码为0XFF 保留
#define ARRAY_READ_FORCE_FIVEFINGER_GRIPPER {0xEB,0x90,0x01,0x04,0x11,0x2E,0x06,0x0C,0xFF}
#define ARRAY_WRITE_MOVESEQID_FIVEFINGER_GRIPPER {0xEB,0x90,0x01,0x04,0x12,0x10,0x09,0x01,0xFF}
#define ARRAY_RUN_MOVESEQID_FIVEFINGER_GRIPPER {0xEB,0x90,0x01,0x04,0x12,0x12,0x09,0x01,0xFF}


/**
 * @brief The fiveFingerSerialGripper class 因时灵巧手
 */
class fiveFingerSerialGripper : public SerialGripper
{
public:
    fiveFingerSerialGripper();

    virtual int getForceVal(std::vector<int>& data) override;
    virtual int setMoveSeq(int index) override;
    virtual int openGripper() override;
    virtual int closeGripper() override;
    virtual int stopGripper() override;
private:
    int bytes2int(unsigned char *array, int lens, int *outArray);
};

//H_DECLARE_PLUGIN(IGripper)
