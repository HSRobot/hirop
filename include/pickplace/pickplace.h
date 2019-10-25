#ifndef PICKANDPLAC_H
#define PICKANDPLAC_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "utils/idebug.h"

#ifdef _PRINT_
#define _COUT_
#endif

namespace hirop_pickplace {

/**
 * @brief 用于生成相关的导出函数，只需传递类名，该宏就会自动生成导出函数
 */
#define HPICKPLACE_MODULE(className) \
extern "C" { \
    void* __create##className(){ \
    return new className;       \
    }\

enum ENTITY_TYPE{
    PYTHON = 0,
    CPP
};

//欧拉角结构体
typedef struct euler{
    double roll;
    double pitch;
    double yaw;
}euler;

//抓取进给方向结构体
typedef struct pick_vect{
    double vect_x;
    double vect_y;
    double vect_z;
}pick_vect;

}
#endif // PICKANDPLAC_H
