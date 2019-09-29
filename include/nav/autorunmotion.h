#pragma once

#include "motion.h"

#include "../msgs/posestamped.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace hirop{

namespace navigation{

class AutoRunMotion :  public Motion{

public:
    AutoRunMotion() : Motion("AutoRunMotion", 1){}

    ~AutoRunMotion();

    int setTargetPose(PoseStamped pose);

    int setup();

    int stop();

    MotionResult update();

private:
    MoveBaseClient *_client;

    move_base_msgs::MoveBaseGoal _goal;

    bool isRunning;

};

}

}
