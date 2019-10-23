#ifndef PICK_EXCUTE_H
#define PICK_EXCUTE_H

#include <string>
#include "configuer.h"
#include "pickplace.h"
#include "cpp_loader.h"
#include "c_base_generator.h"
#include "c_base_pickplace.h"

using std::string;
using namespace std;

namespace hirop_pickplace {

class PickPlace{

public:

    /**
     * @brief 构造函数
     */
    PickPlace();

    /**
     * @brief 析构函数
     */
    ~PickPlace();

    /**
     * @brief 设置生成器
     * @return 0,成功　-1．失败
     */
    int setGenerator(string generatorName,string entityType,std::string configFile);

    /**
     * @brief 设置执行器
     * @return　0,成功　-1．失败
     */
    int setActuator(string actuatorName, string entityType,std::string configFile);

    /**
     * @brief getGeneratorList 获取生成器列表
     * @param generatorList[out] 生成器列表
     * @return
     */
    int getGeneratorList(std::vector<std::string>& generatorList);

    /**
     * @brief getActuatorList 获取执行器列表
     * @param ＆actuatorList[out]　执行器列表
     * @return
     */
    int getActuatorList(std::vector<std::string>& actuatorList);

    /**
     * @brief setPickPose 设置抓取点位
     * @param pickPos[in] 点位
     * @return
     */
    int setPickPose(PoseStamped pickPos);

    /**
     * @brief setPlacePose 设置放置点位
     * @param placePos[in] 点位
     * @return
     */
    int setPlacePose(PoseStamped placePos);

    /**
     * @brief 設置對象位置坐標
     * @return 0,成功　-1．失败
     */
    int showObject(PoseStamped pose);

    /**
     * @brief 移除物体
     * @return　0,成功　-1．失败
     */
    int removeObject();

    /**
     * @brief 运动到点
     * @param pose　点位
     * @return
     */
    int moveToPos(PoseStamped pose);

    /**
     * @brief 运动到定义点
     * @param 点位名称
     * @return
     */
    int moveToFoName(string poseName);

    /**
     * @brief　抓取运行
     * @return　0,成功　-1，失败
     */
    int pick();

    /**
     * @brief 放置运行
     * @return
     * 0,成功
     * -1,失败
     */
    int place();

    /**
     * @brief 停止操作
     * @return
     * 0，成功
     * -1，失败
     */
    int stop();

private:

    /**
     * @brief 具体抓取实现
     * @param 传入，抓取位置
     * @return
     * 0，成功
     * -1，失败
     */
    int _pick(PoseStamped pickPos);

    /**
     * @brief 具体放置实现
     * @param 传入，放置位置
     * @return 0，成功  -1，失败
     */
    int _place(PoseStamped placePos);

    /**
     * @brief　生成抓取位置
     * @param pickPos
     * @return
     */
    int _gen_pickPose(PoseStamped pickPos);

    /**
     * @brief 生成放置位置
     * @param placePos
     * @return
     */
    int _gen_placePose(PoseStamped placePos);

private:
    /**
     * @brief 加载器
     */
    CppLoader *cloaderPtr;

    /**
     * @brief 具体的生成器
     */
    IGenerator *generatorPtr;

    /**
     * @brief 具体的执行器
     */
    IPickPlace *pickplacePtr;

    /**
     * @brief 具体的配置器
     */
    Configure *configuerPtr;

    /**
     * @brief C++ 生成器缓冲器
     */
    map<string, IGenerator*> cppGenerator;

    /**
     * @brief Python 生成器缓冲器
     */
    map<string, IGenerator*> pyGeneratior;

    /**
     * @brief C++ 抓取缓冲器
     */
    map<string, IPickPlace*> cppActuator;

    /**
     * @brief Python 抓取缓冲器
     */
    map<string, IPickPlace*> pyActuator;

    bool stopFlag_;
    PoseStamped objPos_;
};
}
#endif // PICK_EXCUTE_H
