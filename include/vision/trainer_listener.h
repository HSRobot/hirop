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
#ifndef TRAINER_LISTENER_H
#define TRAINER_LISTENER_H

#include <iostream>

namespace hirop_vision {


/**
 * @brief       训练器状态变化监听器接口类
 * @author      XuKunLin
 * @date        2019-03-22
 */
class TrainStateListener{

public:
    /**
     * @brief       当训练器完成训练后，调用该函数
     * @param [trainer] 完成训练的训练器名称
     * @param [ret]     训练器返回结果 0 成功 1 失败
     * @return void
     */
    virtual void onTrainDone(const std::string trainer, int ret) = 0;

    /**
     * @brief       当训练器发生训练进度变化时，调用该函数
     * @param [trainer]     发生进度变化的训练器名称
     * @param [feed]        训练进度
     * @return      void
     */
    virtual void feekback(const std::string trainer, int feed) = 0;

    /**
     * @brief       默认析构函数
     * @param
     * @return
     */
    virtual ~TrainStateListener(){}

};

}

#endif // TRAINER_LISTENER_H
