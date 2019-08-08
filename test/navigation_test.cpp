#include <nav/mobile_robot.h>
#include <nav/linearmotion.h>
#include <nav/circlemotion.h>

using namespace hirop::navigation;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nav_test");

    MobileRobot *robot = MobileRobot::getInstance();

    CircleMotion *cirleMotion = new CircleMotion(2, 0.5);

    LinearMotion *linearMotion = new LinearMotion(1, 0.5);

    robot->runMotion(linearMotion);

    ros::spin();

    return 0;
}
