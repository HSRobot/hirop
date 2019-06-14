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

#ifndef __CONFIGURE_H__
#define __CONFIGURE_H__

#include <iostream>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#ifdef DEBUG
#define configDebug(format, ...)  printf("[debug ]: " #format "\n[detail]: File: %s, Line: %d, Function: %s \n",\
    ##__VA_ARGS__, __FILE__, __LINE__, __FUNCTION__);
#else
#define configDebug(format, ...)
#endif

namespace hirop_vision {

class Configure{

public:
    /**
     * @brief 构造函数
     * @param [file] 输入， yaml配置文件名称
     */
    Configure(std::string file);

    /**
     * @brief       获取配置文件中的物体名称
     * @param    [objName] 输出， 物体名称
     * @return
     *          0 成功
     *          1 失败
     */
    int getObjectName(std::string &objName);

    /**
     * @brief       获取配置文件中训练器名称
     * @param   [trainerName] 输出，训练器名称
     * @return
     *          0 成功
     *          1 失败
     */
    int getTrainerName(std::string &trainerName);

    /**
     * @brief getPrivateConfig  获取训练器私有参数
     * @return
     *          0 成功
     *          -1 无私有参数
     */
    int getPrivateParams(YAML::Node &privateParams);

private:
    // 当前Configuere中的yaml配置文件实例
    YAML::Node config;

};

}

#endif
