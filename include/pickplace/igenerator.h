#ifndef IPOSES_BUILDER_H
#define IPOSES_BUILDER_H

#include <yaml-cpp/yaml.h>
#include "msgs/posestamped.h"
#include "hplugin.h"
#include "pickplace.h"

using namespace hirop;

namespace hirop_pickplace{

class IGenerator{

public:

    /**
     * @brief 设置抓取的目标点位
     * @return  0，成功  -1，失败
     */
    virtual int setPickPose(PoseStamped)  = 0;

    /**
     * @brief 设置放置的目标点位
     * @return 0，成功  -1，失败
     */
    virtual int setPlacePose(PoseStamped) = 0;

    /**
     * @brief 生成抓取的目标点位
     * @return 0，成功  -1，失败
     */
    virtual int genPickPose() = 0;

    /**
     * @brief 生成放置的目标点位
     * @return 0，成功  -1，失败
     */
    virtual int genPlacePose() = 0;

    /**
     * @brief 获取生成后的抓取点位
     * @param pickPoses
     * @return
     */
    virtual int getResultPickPose(PoseStamped&) = 0;

    /**
     * @brief 获取生成后的放置点位
     * @param placePoses
     * @return
     */
    virtual int getResultPlacePose(PoseStamped&) = 0;

    /**
     * @brief 解析私有参数
     * @return
     */
    virtual int parseConfig(YAML::Node& ) = 0;

    /**
     * @brief 停止生成
     * @return 0，成功  -1，失败
     */
    virtual int stopGenerator() = 0;

    virtual int getName(std::string &name) = 0;

    virtual ENTITY_TYPE getEntityType() = 0;

     virtual ~IGenerator(){}

protected:
    // 保存识别器的字符描述
    std::string name;

    // 其算法实现类别 C++ / Python
    hirop_pickplace::ENTITY_TYPE entityType;

};
}
H_DECLARE_INTERFACE(hirop_pickplace::IGenerator,"IGenerator/v1.0")


#endif // IPOSES_BUILDER_H
