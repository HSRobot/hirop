#ifndef POSES_BUILDER_H
#define POSES_BUILDER_H
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include "pickplace/igenerator.h"
#include "pickplace/c_base_generator.h"
#include "hpluginloader.h"


#define PI  3.14159265358979323846   // pi

using namespace hirop_pickplace;

class ClassicGenerator:public CBaseGenerator{

struct Sparameter{
    bool sorting;

    bool follow_R;
    bool follow_P;
    bool follow_Y;

    double dir_R;
    double dir_P;
    double dir_Y;

    float offset_x;
    float offset_y;
    float offset_z;

    double p_dir_R;
    double p_dir_P;
    double p_dir_Y;

    float p_offset_x;
    float p_offset_y;
    float p_offset_z;
}parameter;

public:

    /**
     * @brief 构造函数
     */
    ClassicGenerator();

    /**
     * @brief 析构函数
     */
    ~ClassicGenerator();

    /**
     * @brief 设置抓取位姿
     * @return 1 成功， -1 失败
     */
    int setPickPose(PoseStamped ) ;

    /**
     * @brief 设置放置位姿
     * @return 0 成功， -1 失败
     */
    int setPlacePose(PoseStamped ) ;

    /**
     * @brief 执行生成抓取点位操作
     * @return 1 成功， -1 失败
     */
    int genPickPose() ;

    /**
     * @brief 执行生成放置点位操作
     * @return 0成功， -1 失败
     */
    int genPlacePose() ;

    /**
     * @brief 获取生成后的抓取点位
     * @param pickPoses
     * @return
     */
    int getResultPickPose(PoseStamped& );

    /**
     * @brief 获取生成后的放置点位
     * @param placePoses
     * @return
     */
    int getResultPlacePose(PoseStamped& );

    /**
     * @brief 停止生成点位
     * @return 0 成功， -1 失败
     */
    int stopGenerator() ;

    /**
     * @brief 获取私有参数
     * @param 参数节点
     * @return
     */
    int parseConfig(YAML::Node& );

private:

    /**
     * @brief 调整欧拉角
     * @return
     */
    int correctEuler(euler, euler&);

    /**
     * @brief 四元数转欧拉角
     * @param 输入欧拉角
     * @param 输出四元数
     * @return 0 成功， -1 失败
     */
    int quaternionToEuler(Quaternion quat,euler& euler);

    /**
     * @brief 欧拉角转四元素
     * @param euler_angle，输入欧拉角
     * @param quat ，输出，四元素
     * @return 0
     */
    int eulerToQuaternion(euler euler, Quaternion& quat);

    int quatPro(Quaternion quat1, Quaternion quat2, Quaternion &quat3);

    int quatFromVector(double vec[4], Quaternion& quat);

    int quatConvect(Quaternion &quatConvect);

//    template<typename T>
//    T getParam(const YAML::Node& ndoe, const std::string& name, T& defaultValue);

private: 
   Sparameter m_parm;
   PoseStamped m_pickpose;
   PoseStamped m_placepose;
   euler _pick_euler;

};

H_DECLARE_PLUGIN(IGenerator)
#endif // POSES_BUILDER_H
