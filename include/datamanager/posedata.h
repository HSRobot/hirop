#pragma once

#include "hdata.h"
#include "../msgs/posestamped.h"

namespace hirop{

namespace data_manager{

class PoseData : public HData{

public:

    PoseData() : HData("pose"){}

    hirop::PoseStamped pose;

private:
    /**
     *  声明需要被序列化的对象
     */
    START_NEED_SAVE_OR_LOAD()
    VAR_NEED_SAVE_OR_LOAD(pose.pose.orientation.x);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.orientation.y);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.orientation.z);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.orientation.w);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.position.x);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.position.y);
    VAR_NEED_SAVE_OR_LOAD(pose.pose.position.z);
    VAR_NEED_SAVE_OR_LOAD(pose.frame_id);
    END_NEED_SAVE_OR_LOAD()

};

}

}
