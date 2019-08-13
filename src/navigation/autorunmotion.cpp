#include <nav/autorunmotion.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace hirop::navigation;

AutoRunMotion::~AutoRunMotion(){

}

int AutoRunMotion::setTargetPose(PoseStamped pose){

    _goal.target_pose.header.frame_id = pose.frame_id;

    HIROP_POSE_2_ROS_POSE(pose.pose, _goal.target_pose.pose);

}

int AutoRunMotion::stop(){
    return 0;
}

int AutoRunMotion::setup(){

    /**
     *  初始化Action
     */

    _client =  new MoveBaseClient("move_base", true);

    if(_client->waitForServer()){
        return 0;
    }

    return -1;
}

Motion::RETURN_TYPE AutoRunMotion::update(){

    _client->sendGoal(_goal);

    _client->waitForResult();

    return FINISHED;
}
