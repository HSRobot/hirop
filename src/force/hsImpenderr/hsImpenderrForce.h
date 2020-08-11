/** 
* @version 	v1.0.0
* @file		hsImpenderr.cpp
* @brief	力控阻抗算法
* @autor 	KongDeLiang
* @date		2020/8/07
*/

#pragma once
//#include <force/forceCppBase .h>
#include <force/iForce.h>
#include <iostream>
using namespace std;
using namespace hirop_force;

class hsImpenderrForce : public IForce{

public:
    hsImpenderrForce();
    ~hsImpenderrForce();

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


    ENTITY_TYPE getEntityType()
    {
        return entityType;
    }
private:

    /**
     * @brief setDirection 设置力控的方向模式
     * @param name
     * @return
     */
    int setDirection(int *direction);


    /**
     * @brief swithMode  切换模式
     * @param type 0 关节 1 笛卡尔模式
     * @return
     */
    int swithMode(int type);

    void transformVecStr2Array(double *arr, int size, vector<string> & data);

    void transformVecStr2Array(int *arr, int size, vector<string> &data);

private:
    YAML::Node  node;
    string pluginName;

private:
    vector<double> Force; // sensor value
    vector<double> robotPose; // sensor value

    double m_Stiffness[6];		//刚性
    double m_Damping[6];		//阻尼
    double ts;
    double dXa[6];

    double *dXa_1;
    double *Xa_1;
    double *Xa;
    double *ddXa;

    double m_Mass[6];		//质量
    int direction[6];
};



H_DECLARE_PLUGIN(IForce)
