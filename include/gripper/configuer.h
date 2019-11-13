#ifndef CONFIGUER_H
#define CONFIGUER_H
#include <iostream>
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <utils/idebug.h>

#ifdef DEBUG
#define configDebug(format, ...)  printf("[debug ]: " #format "\n[detail]: File: %s, Line: %d, Function: %s \n",\
    ##__VA_ARGS__, __FILE__, __LINE__, __FUNCTION__);
#else
#define configDebug(format, ...)
#endif

namespace hirop_gripper {

class Configure{

public:
    /**
     * @brief 构造函数
     * @param [file] 输入， yaml配置文件名称
     */
    Configure(std::string fileName);

    /**
     * @brief       获取配置文件中夹爪的名称
     * @param   [trainerName] 输出，生成器名称
     * @return
     *          0 成功
     *          1 失败
     */
    int getGripperName(std::string& generatorName);

    /**
     * @brief 获取私有参数
     * @return
     * 0 ,有私有参数
     * -1,无私有参数
     */
    int getPrivateParams(YAML::Node& yamlNode);

private:
    // 当前Configuere中的yaml配置文件实例
    YAML::Node config;
};

}
#endif // CONFIGUER_H
