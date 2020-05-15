#include "serial_gripper.h"
#include "gtest/gtest.h"
#include "hirop/gripper/configuer.h"



TEST(SERIAL, READ)
{
    SerialGripper s;
//    s.parseConfig();
    YAML::Node privateParam;
    Configure *config = new Configure("/home/de/catkin_ws/src/hirop_ros/gripper_bridge/config/GripperConfig0.yaml");
    config->getPrivateParams(privateParam);

    s.parseConfig(privateParam);
    s.connectGripper();
//    s.openGripper();
    s.isConnectGripper();
    for(int i = 0; i< 100;i++){
        s.closeGripper();
        usleep(1000);
        int data = s.readGripperCurrenPose();
        std::cout << "close "<<data<<std::endl;

        s.openGripper();
        data = s.readGripperCurrenPose();

        std::cout << "open "<<data<<std::endl;

        usleep(1000);
    }
//    s.closeGripper();
//    int data = s.readGripperCurrenPose();
//    std::cout <<data<<std::endl;
}

int main(int argc,char **argv)
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();

}
