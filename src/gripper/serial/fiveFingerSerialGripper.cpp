#include "fiveFingerSerialGripper.h"


fiveFingerSerialGripper::fiveFingerSerialGripper(){}

int fiveFingerSerialGripper::getForceVal(std::vector<int> &data)
{
    unsigned char buffer[] = ARRAY_READ_FORCE_FIVEFINGER_GRIPPER;

    int size_buffer = ARRAY_SIZE(buffer);
    buffer[size_buffer-1] = getCheckNum(buffer,size_buffer);

    ros_ser.write(buffer,size_buffer);
    IDebug("Read getForceVal!!");

    unsigned char read_buffer[20];
    int *force = (int *)malloc(6);
    ros_ser.flushInput();
    int size_close_buffer = 20;
    size_t t =ros_ser.read(read_buffer,size_close_buffer);
    int size =static_cast<int>(t);
    if(size != size_close_buffer){
        IDebug(" Receive from size is not good\n");
        return -1;
    }
    bytes2int(read_buffer,6,force);

    data.clear();
    data.resize(6);
    for(int i = 0; i < 6; i++){
        data[i] = (force[i]);
    }

    free(force);

    return 0;
}

int fiveFingerSerialGripper::setMoveSeq(int index)
{
    size_t size = 0;
    {
        unsigned char buffer[] = ARRAY_WRITE_MOVESEQID_FIVEFINGER_GRIPPER;

        int size_buffer = ARRAY_SIZE(buffer);

        //run id
        buffer[size_buffer-2] = index;
        //mask
        buffer[size_buffer-1] = getCheckNum(buffer,size_buffer);

        ros_ser.write(buffer,size_buffer);
        IDebug("write setMoveSeq!!");

        unsigned char read_buffer[9];
        int size_close_buffer = ARRAY_SIZE(read_buffer);
        size = ros_ser.read(read_buffer,size_close_buffer );
    }

    if(size != 9)
        return -1;
    /**********************************/
    {
        unsigned char buffer[] = ARRAY_RUN_MOVESEQID_FIVEFINGER_GRIPPER;

        int size_buffer = ARRAY_SIZE(buffer);
        buffer[size_buffer-1] = getCheckNum(buffer,size_buffer);

        ros_ser.write(buffer,size_buffer);
        IDebug("Read getForceVal!!");

        unsigned char read_buffer[9];
        int size_close_buffer = ARRAY_SIZE(read_buffer);
        size = ros_ser.read(read_buffer,size_close_buffer );
    }

    if(size != 9)
        return -2;

    return 0;

}

int fiveFingerSerialGripper::openGripper()
{
    return 0;
}

int fiveFingerSerialGripper::closeGripper()
{
    return 0;
}

int fiveFingerSerialGripper::stopGripper()
{
    return 0;
}


/*
    for i in range(1,7):
            if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
                actforce[i-1] = -1
            else:
                actforce[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
        return actforce
*/
int fiveFingerSerialGripper::bytes2int(unsigned char * array, int lens,\
                                              int * outArray)
{
    for(int i = 0; i< lens  ; i++)
    {
        if(array[(i+1)*2 + 5] ==0xFF && (array[ (i+1)*2 + 6] ==0xFF ))
             outArray[i] = -1;
        else
            outArray[i] = (int)(array[(i+1)*2 + 5] + ((array[ (i+1)*2 + 6])<<8));

        if(outArray[i] > 2000)
            outArray[i] -=65535;
    }
}
H_EXPORT_PLUGIN(fiveFingerSerialGripper,  "fiveFingerSerialGripper",  "1.0")
