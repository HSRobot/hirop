#include "serial_gripper.h"
#include "gtest/gtest.h"
#include "hirop/gripper/configuer.h"
#include "fiveFingerSerialGripper.h"


TEST(SERIAL, READ)
{
    fiveFingerSerialGripper s;
//    s.parseConfig();
    YAML::Node privateParam;
    Configure *config = new Configure("/home/maizitian/catkin_ws/src/hirop_ros/gripper_bridge/config/GripperConfig0.yaml");
    config->getPrivateParams(privateParam);

    int ret =s.parseConfig(privateParam);
    ASSERT_EQ(0 ,ret);
    s.connectGripper();
    s.openGripper();
    ASSERT_EQ(true,s.isConnectGripper());
    std::vector<int> data;
    while(true){
        ASSERT_EQ(0,s.getForceVal(data));
        for(auto it : data)
        {
            std::cout <<it<<" ";
        }
        std::cout <<std::endl;
    }
//    s.setMoveSeq(1);

//    for(int i = 0; i< 100;i++){
//        s.closeGripper();
//        usleep(1000);
//        int data = s.readGripperCurrenPose();
//        std::cout << "close "<<data<<std::endl;

//        s.openGripper();
//        data = s.readGripperCurrenPose();

//        std::cout << "open "<<data<<std::endl;

//        usleep(1000);
//    }

}

int main(int argc,char **argv)
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();

}
