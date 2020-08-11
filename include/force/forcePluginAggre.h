#pragma once
#include <force/iForce.h>
#include <force/forceExport.h>
#include <force/IForceLoader.h>
#include <boost/shared_ptr.hpp>
using namespace hirop_force;
using namespace std;

class forcePluginAggre
{
public:
    forcePluginAggre();
    ~forcePluginAggre();

    /**
     * @brief setForcePlugin
     * @param generatorName
     * @param entityType
     * @param configFile
     * @return
     */
    int setForcePlugin(string generatorName,string entityType,std::string configFile);

    /**
     * @brief getForcePluginList 获取夹爪列表
     * @param forceList[out] 夹爪列表
     * @return
     */
    int getForcePluginList(std::vector<std::string>& forceList);


    /**
     * @brief 解析私有参数
     * @return
     */
    int parseConfig(YAML::Node& node);

    /**
     * @brief setInputForce 输入力矩参数
     * @return
     */
    void setInputForceBias(std::vector<double> &pose);

    /**
     * @brief setInputRobotPose 输入机器人的关节坐标
     * @return
     */
    void setInputRobotPose(std::vector<double> &pose);

    /**
     * @brief compute 算法运算
     * @return
     */
    int compute();

    /**
     * @brief getResult 获取力控运算的输出结果
     * @return
     */
    int getResult(std::vector<double> &pose);

    /**
     * @brief getName  获取算法插件的名称
     * @param name
     * @return
     */
    std::string  getName();


    /**
     * @brief printInfo 输出打印信息
     * @return
     */
    int printInfo();
private:
    std::map<std::string, boost::shared_ptr<IForce>> cppforcePtrMap;
    boost::shared_ptr<IForce> forcePtr;
    boost::shared_ptr<IForceLoader> forceCppLoadPtr;
};


