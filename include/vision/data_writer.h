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
#ifndef __DATA_WIRTER_H__
#define __DATA_WIRTER_H__

namespace hirop_vision {

/**
 * @brief       负责对训练信息进行保存
 * @author      XuKunLin
 * @date        2019-03-20
 */
class DataWiter{

public:

    /**
     * @brief   添加新的数据
     * @return
     *          0   设置成功
     *          -1  设置失败
     */
    int add();

    /**
     * @brief   删除数据
     * @return
     *          0   设置成功
     *          -1  设置失败
     */
    int del();

    /**
     * @brief   查找数据
     * @return
     *          0   设置成功
     *          -1  设置失败
     */
    int serach();

    /**
     * @brief   修改数据
     * @return
     *          0   设置成功
     *          -1  设置失败
     */
    int change();

private:
    int create();

};

}

#endif
