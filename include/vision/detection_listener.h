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
#ifndef DETECTION_LISTENER_H
#define DETECTION_LISTENER_H

#include "vision.h"
#include <iostream>
#include <vector>

namespace hirop_vision {

/**
 * @brief       检测器状态变化监听器。
 * @author      XuKunLin
 * @date        2019-03-22
 */
class DetectStateListener{

public:
    /**
     * @brief       当检测器状态发生变化后，调用该函数
     * @param [detector]    发生状态变化的检测器名称
     * @return      void
     */
    virtual void onDetectDone(std::string detector, int ret, std::vector<pose> p) = 0;
};

}

#endif // DETECTION_LISTENER_H
