#pragma once

#include <iostream>

namespace hirop {

#define ROS_POSE_2_HIROP_POSE(ROS_POSE, HIROP_POSE) \
    HIROP_POSE.position.x = ROS_POSE.position.x; \
    HIROP_POSE.position.y = ROS_POSE.position.y; \
    HIROP_POSE.position.z = ROS_POSE.position.z; \
    HIROP_POSE.orientation.x = ROS_POSE.orientation.x; \
    HIROP_POSE.orientation.y = ROS_POSE.orientation.y; \
    HIROP_POSE.orientation.z = ROS_POSE.orientation.z; \
    HIROP_POSE.orientation.w = ROS_POSE.orientation.w;

#define HIROP_POSE_2_ROS_POSE(HIROP_POSE, ROS_POSE) ROS_POSE_2_HIROP_POSE(HIROP_POSE, ROS_POSE)

typedef struct Point3D{

    float x;
    float y;
    float z;

} Point3D;

typedef struct Quaternion{

    float x;
    float y;
    float z;
    float w;

} Quaternion;

typedef struct Pose{

    Point3D position;
    Quaternion orientation;

} Pose;

}
