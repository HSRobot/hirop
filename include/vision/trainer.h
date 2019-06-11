#ifndef __ITRAINER_H__
#define __ITRAINER_H__

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
#include "configure.h"
#include "itrainer.h"
#include "trainer_listener.h"

#include <boost/thread.hpp>
#include "cpp_loader.h"

namespace hirop_vision {

/**
 * @brief           hirop_vision的training接口类，使用该类可以进行相关的物体识别等接口
 * @author          XuKunLin
 * @date            2019-03-20
 * @todo            1，增加状态枚举
 *                  2，实现feedback机制
 *                  3，增加析构函数，回收内存
 */
class Trainer
{

typedef void (*CBFUN)(int, std::string);

public:

    /**
     * @brief   默认构造函数
     */
    Trainer();

    /**
     * @brief   训练接口，调用该接口则开始进行训练
     * @return
     *          0 开始训练成功
     *          -1 开始训练失败
     */
    int train();

    /**
     * @brief   传递训练配置文件
     * @param [fileName] 配置文件名称。 可指定路径
     * @return
     *          0  传递成功
     *          -1 传递失败
     */
    int setTrainConfig(std::string fileName);

//    /**
//     * @brief   列出当前系统中已训练的所有物体
//     * @return  物体列表
//     */
//    int listObject();

//    /**
//     * @brief   删除指定物体
//     * @return  物体列表
//     */
//    int deteObject();

//    /**
//     * @brief   列出当前系统中已加载的训练器
//     * @return  物体列表
//     */
//    int listTrain();


    /**
     * @brief       设置状态变换后的监听者
     * @param
     * @return
     */
    int setOnStateChangeListener(TrainStateListener *listener);

private:

    /**
     * @brief   执行具体的训练任务
     * @return
     *          0 开始训练成功
     *          -1 开始训练失败
     */
    int __train();

    /**
     * @brief       生成文件保存路径
     * @param  [path] 输出 生成的路径
     * @return
     *          0   成功
     *          -1   失败
     */
    int __genPath(std::string &path);


private:
    // 训练线程
    boost::thread *trainThr;

    // 具体的训练器
    hirop_vision::ITrainer *trainer;

    // 当前的配置信息
    Configure *config;

    // 动态库加载器
    CppLoader *loader;

    // 训练状态变换监听者
    TrainStateListener *listener;
};
}


#endif
