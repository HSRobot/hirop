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

#ifndef __C_BASE_TRAINER_H__
#define __C_BASE_TRAINER_H__

#include "itrainer.h"

namespace hirop_vision {

class CBaseTrainer:public ITrainer{
public:
    /**
     * @brief       构造函数
     * @param   [trainerName] 训练器的名称
     */
    CBaseTrainer(std::string trainerName);

    /**
     * @brief       获取训练器的名称
     * @param   [name] 返回训练器的名称
     * @return  0 成功 1 失败
     */
    int getName(std::string &name);

    /**
     * @brief      获取当前训练器的实现实体
     * @return      返回训练器的实现实体类别
     */
    ENTITY_TYPE getEntityType();
};

}

#endif
