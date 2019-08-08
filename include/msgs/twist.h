#ifndef __TWIST_H__
#define __TWIST_H__

namespace hirop {

#define ROS_TWIST_2_HIROP_TWIST(ROS_TWIST, HIROP_TWIST) \
    HIROP_TWIST.linear.x = ROS_TWIST.linear.x; \
    HIROP_TWIST.linear.y = ROS_TWIST.linear.y; \
    HIROP_TWIST.linear.z = ROS_TWIST.linear.z; \
    HIROP_TWIST.angular.x = ROS_TWIST.angular.x; \
    HIROP_TWIST.angular.y = ROS_TWIST.angular.y; \
    HIROP_TWIST.angular.z = ROS_TWIST.angular.z;

#define HIROP_TWIST_2_ROS_TWIST(HIROP_TWIST, ROS_TWIST) \
    ROS_TWIST_2_HIROP_TWIST(HIROP_TWIST, ROS_TWIST)

typedef struct Linear{

    float x;
    float y;
    float z;

} Linear;

typedef struct Angular{
    float x;
    float y;
    float z;
} Angular;

typedef struct Twist{

    Linear linear;
    Angular angular;

} Twist;

}

#endif
