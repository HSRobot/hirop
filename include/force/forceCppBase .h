#pragma once
#include <force/iForce.h>
namespace hirop_force{

class forceCppBase: public IForce
{
public:
    forceCppBase();
    ~forceCppBase();
    /**
     * @brief 解析私有参数
     * @return
     */
    virtual int parseConfig(YAML::Node& node);

    /**
     * @brief setInputForce 输入力矩参数
     * @return
     */
    virtual void setInputForceBias(std::vector<double> &value);

    /**
     * @brief setInputRobotPose 输入机器人的笛卡尔坐标
     * @return
     */
    virtual void setInputRobotCartPose(std::vector<double> &value);

    /**
     * @brief compute 算法运算
     * @return
     */
    virtual int compute();

    /**
     * @brief getResult 获取力控运算的输出结果
     * @return
     */
    virtual int getResult(std::vector<double> &pose);

    /**
     * @brief getName  获取算法插件的名称
     * @param name
     * @return
     */
    virtual std::string getName();


    /**
     * @brief printInfo 输出打印信息
     * @return
     */
    virtual int printInfo();

    /**
     * @brief setDirection 设置力控的方向模式
     * @param name
     * @return
     */
//    virtual int setDirection(std::string &name);
};

}
