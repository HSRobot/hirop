#ifndef IPICKPLACE_H
#define IPICKPLACE_H

#include <yaml-cpp/yaml.h>
#include "msgs/posestamped.h"
#include "hplugin.h"
#include "pickplace.h"

#define __LINUX__

using namespace hirop;

namespace hirop_pickplace {

class IPickPlace{
public:

    /**
     * @brief 解析私有参数
     * @return
     */
    virtual int parseConfig(YAML::Node& ) = 0;

    /**
     * @brief 设置抓取点位
     * @param 传入，抓取点位
     * @return 0，成功  -1，失败
     */
    virtual int setPickPose(PoseStamped ) = 0;

    /**
     * @brief 设置放置点位
     * @param 传入，放置点位
     * @return 0，成功  -1，失败
     */
    virtual int setPlacePose(PoseStamped ) = 0;

    /**
     * @brief 显示物体
     * @return 0，成功  -1，失败
     */
    virtual int showObject(PoseStamped) = 0;

    /**
     * @brief 删除显示物体
     * @return
     * 0,成功
     * -1,失败
     */
    virtual int removeObject() = 0;

     /**
     * @brief 运动到点
  　 * @param 点位
     * @return
     */
    virtual int moveToPos(PoseStamped) = 0;

    /**
     * @brief 运动到定义点

     * @return
     */
    virtual int moveToFoName(std::string) = 0;

    /**
     * @brief 抓取实现
     * @return 0，成功  -1，失败
     */
    virtual int pick() = 0;

    /**
     * @brief 放置实现
     * @return 0，成功  -1，失败
     */
    virtual int place() = 0;

    /**
     * @brief 执行器停止
     * @return 0，成功  -1，失败
     */
    virtual int stopPickplace() = 0;

    virtual int getName(std::string &name) = 0;

    virtual ENTITY_TYPE getEntityType() = 0;

    virtual ~IPickPlace(){}

protected:
    // 保存器的字符描述
    std::string name;

    // 其算法实现类别 C++ / Python
    hirop_pickplace::ENTITY_TYPE entityType;
};

}
H_DECLARE_INTERFACE(hirop_pickplace::IPickPlace,"IPickPlace/v1.0")
#endif // IPICKPLACE_H
