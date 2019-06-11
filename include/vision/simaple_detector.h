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
#ifndef __SIMAPLE_DETECTOR_H__
#define __SIMAPLE_DETECTOR_H__

#include "idetector.h"

using namespace hirop_vision;

class SimapleDetector:public IDetector{


public:
    SimapleDetector();

    int detection();

    int loadData(std::string objectName);

    int getResult(pose &p);

};

#endif // SIMAPLE_DETECTOR_H
