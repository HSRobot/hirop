#pragma once

#include "pose.h"
#include "twist.h"

namespace hirop {

/**
 *  将ROS的odom装换为HIROP的odom数据类型
 */
#define ROS_ODOM_2_HIROP_ODOM(ROS_ODOM, HRIOP_ODOM) \
    ROS_POSE_2_HIROP_POSE(ROS_ODOM.pose.pose, HRIOP_ODOM.pose)

/**
 *  将ROS的odom装换为HIROP的odom数据类型
 */
#define HIROP_ODOM_2_ROS_ODOM(HRIOP_ODOM, ROS_ODOM) \
    HIROP_POSE_2_ROS_POSE(HRIOP_ODOM.pose, ROS_ODOM.pose.pose)

struct Odometry{

    Pose pose;
    Twist twist;

};

}
