/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 *
 * Author: Kunlin Xu <1125290220@qq.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 */
#ifndef __VISION_H__
#define __VISION_H__

#include <iostream>

#define OBJECT_DATA_PREFIX "hirop_vision/data"

namespace hirop_vision {

/**
 * @brief 用于生成相关的导出函数，只需传递类名，该宏就会自动生成导出函数
 */
#define HVISION_MODULE(className) \
extern "C" { \
    void* __create##className(){ \
    return new className;       \
    }\
}

enum ENTITY_TYPE{
    PYTHON = 0,
    CPP
};

typedef struct quaternion{
    double x;
    double y;
    double z;
    double w;
} quaternion;

typedef struct position{
    double x;
    double y;
    double z;
} position;

typedef struct pose{
    std::string objectName;
    struct quaternion quaternion;
    struct position position;
} pose;

inline std::string getObjectDataPath(){
    std::string homePath = getenv("HOME");
    return  homePath + "/" +  OBJECT_DATA_PREFIX + "/";
}

}
#endif
