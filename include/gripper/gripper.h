#ifndef GRIPPER_H
#define GRIPPER_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "utils/idebug.h"

#ifdef _PRINT_
#define _COUT_
#endif

namespace hirop_gripper {

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

}
#endif // GRIPPER_H
