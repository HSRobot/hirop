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
#ifndef __TRAINER_BASE_H__
#define __TRAINER_BASE_H__

#include <iostream>
#include "vision.h"
#include "hplugin.h"

namespace hirop_vision{

/**
 * @brief       具体训练算法训练器基类
 * @author      XuKunLin
 * @date        2019-03-19
 */
class ITrainer{

public:
    /**
     * @brief   实现具体的训练功能，需要具体的训练器实现
     * @return  0 训练成功 1 训练失败
     */
    virtual int train() = 0;

    /**
     * @brief   解析用户传递给训练器的配置，具体由具体的训练器进行实现
     * @return void
     */
    virtual int parseConfig() = 0;

    /**
     * @brief   保存训练器的结果，由具体的训练器进行实现
     * @return void
     */
    virtual int saveData(std::string path) = 0;

    /**
     * @brief   删除对应的训练结果，由具体的训练器进行实现
     * @return void
     */
    virtual int deleteData() = 0;

    /**
     * @brief   获取当前训练进度
     *
     * 如果具体训练器可以获取训练的进度，则应该实现该方法。
     * 该方法将有framewrok调用
     *
     * @return
     *         0 <= 返回当前训练的百分比
     *         -1 错误
     */
    virtual int feedback() = 0;

    /**
     * @brief   停止当前训练
     *
     * 具体的训练器，应当实现该方法。该函数在用户停止训练时由framework调用
     * 具体的训练器应当在这个方法中实现对训练器一些变量、参数的保存和清理工作，如果用户不实现改方法
     * framework将会采用停止线程的方式终止训练。
     *
     * @return
     *          0 成功
     *          -1 失败
     */
    //virtual int stopTrain();

    /**
     * @brief   获取当前训练器的名称
     * @param [name] 输出当前训练器的名称
     * @return
     *          0 开始训练成功
     *          -1 开始训练失败
     */
    virtual int getName(std::string &name) = 0;


    /**
     * @brief      获取当前训练器的实现实体
     * @return      返回训练器的实现实体类别
     */
    virtual ENTITY_TYPE getEntityType() = 0;

    /**
     * @brief       析构函数
     * @param
     * @return
     */
    virtual ~ITrainer(){}

protected:
    // 训练器的字符描述
    std::string name;

    // 训练器的私有数据
    void *privData;

    // 其算法实现类别 C++ / Python
    hirop_vision::ENTITY_TYPE entityType;

};

}
H_DECLARE_INTERFACE(hirop_vision::ITrainer, "ITrainer/V1.0")
#endif
