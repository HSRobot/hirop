#ifndef EXCUTE_H
#define EXCUTE_H

#include <string>
#include "configuer.h"
#include "gripper.h"
#include "cpp_loader.h"
#include "c_base_gripper.h"

using std::string;
using namespace std;

namespace hirop_gripper {

class Gripper{

public:

    /**
     * @brief 构造函数
     */
    Gripper();

    /**
     * @brief 析构函数
     */
    ~Gripper();

    /**
     * @brief 设置夹爪
     * @return 0,成功　-1．失败
     */
    int setGripper(string generatorName,string entityType,std::string configFile);

    /**
     * @brief getGripperList 获取夹爪列表
     * @param gripperList[out] 夹爪列表
     * @return
     */
    int getGripperList(std::vector<std::string>& generatorList);

    /**
     * @brief connectGripper 连接夹爪
     * @return
     */
    int connectGripper();

    /**
     * @brief isConnectGripper 夹爪是否连接
     * @return
     */
    bool isConnectGripper();

    /**
     * @brief disConnectGripper 关闭夹爪连接
     * @return
     */
    int disConnectGripper();

    /**
     * @brief openGripper 打开夹爪
     * @return
     */
    int openGripper();

    /**
     * @brief CloseGripper 关闭夹爪
     * @return
     */
    int closeGripper();

    /**
     * @brief stopGripprt 夹爪急停
     * @return
     */
    int stopGripprt();

    /**
     * @brief getForceVal
     * @param data
     * @return
     */
    int getForceVal(std::vector<int>& data);

    /**
     * @brief setMoveSeq
     * @param index
     * @return
     */
    int setMoveSeq(int index);
private:
    /**
     * @brief 加载器
     */
    CppLoader *cloaderPtr;

    /**
     * @brief 具体的生成器
     */
    IGripper *gripperPtr;

    /**
     * @brief 具体的配置器
     */
    Configure *configuerPtr;

    /**
     * @brief C++ 生成器缓冲器
     */
    map<string, IGripper*> cppGripper;

    /**
     * @brief Python 生成器缓冲器
     */
    map<string, IGripper*> pyGripper;
};
}
#endif // EXCUTE_H
