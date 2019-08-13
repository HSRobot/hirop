#pragma once

#include "pose.h"

namespace hirop{

typedef struct{

    std::string frame_id;
    Pose pose;

} PoseStamped;

}
