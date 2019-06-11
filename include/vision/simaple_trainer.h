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
#ifndef __SIMAPLE_TRAINER_H__
#define __SIMAPLE_TRAINER_H__

#include "c_base_trainer.h"

using namespace hirop_vision;

/**
 * @brief         测试使用的简单训练器
 * @author        XuKunLin
 * @date          2019-03-20
 */
class SimapleTrainer:public CBaseTrainer{

private:
    int feed;
    int objectName;

public:
    /**
     * @brief   构造函数
     */
    SimapleTrainer();

    /**
     * @brief   虚函数实现，详见 itrainer.h
     */
    int train();

    /**
     * @brief   虚函数实现，详见 itrainer.h
     */
    int saveData(std::string path);

    /**
     * @brief   虚函数实现，详见 itrainer.h
     */
    int feedback();

    /**
     * @brief   虚函数实现，详见 itrainer.h
     */
    int parseConfig();

    /**
     * @brief   虚函数实现，详见 itrainer.h
     */
    int deleteData();

};

#endif // SIMAPLE_TRAINER_H
