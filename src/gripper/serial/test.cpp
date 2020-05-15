#include "serial_gripper.h"
#include "gtest/gtest.h"
#include "hirop/gripper/configuer.h"

TEST(SERIAL, READ)
{
    SerialGripper s;
//    s.parseConfig();
    YAML::Node privateParam;
    Configure *config = new Configure("1.yaml");
    config->getPrivateParams(privateParam);

    s.parseConfig(privateParam);
    s.connectGripper();
    s.readGripperCurrenPose();
    std::cout << "ok"<<std::endl;
}

int main(int argc,char **argv)
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();

}
