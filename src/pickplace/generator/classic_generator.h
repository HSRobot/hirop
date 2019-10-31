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

    /**
     * @brief quatPro,四元数旋转
     * @param quat1,旋转原始四元数
     * @param quat2,旋转向量四元数
     * @param quat3，旋转后的四元数
     * @return
     */
    int quatPro(Quaternion quat1, Quaternion quat2, Quaternion &quat3);

    /**
     * @brief quatFromVector,由向量得到四元数
     * @param vec,向量数组，分别为　x,y,z,旋转角度
     * @param quat[out] 旋转向量四元数
     * @return
     */
    int quatFromVector(double vec[4], Quaternion& quat);

    /**
     * @brief tfQuatRotation 由TF库实现旋转
     * @param vx 旋转向量x分量
     * @param vy 旋转向量y分量
     * @param vz 旋转向量z分量
     * @param tan 旋转角度
     * @param quatOrigin，旋转前四元数
     * @return
     */
    Quaternion tfQuatRotation(double vx, double vy, double vz, double tan, Quaternion quatOrigin);

    /**
     * @brief setQuaternion 生成Quaternion类型四元数
     * @param qx　四元数x值
     * @param qy　四元数y值
     * @param qz　四元数z值
     * @param qw　四元数w值
     * @return
     */
    Quaternion setQuaternion(float qx, float qy, float qz, float qw);

    /**
     * @brief quatConvect  四元数转换
     * @param quat　转换前四元数
     * @param quatConvect　转化后四元数
     * @return
     */
    int quatConvect(Quaternion quat, Quaternion &quatConvect);

    /**
     * @brief tfQuatConvect tf实现四元数转换
     * @return
     */
    int tfQuatConvect();

private: 
   Sparameter m_parm;
   PoseStamped m_pickpose;
   PoseStamped m_placepose;
   euler _pick_euler;
};

H_DECLARE_PLUGIN(IGenerator)
#endif // POSES_BUILDER_H
