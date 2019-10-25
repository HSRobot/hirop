#ifndef PICK_EXCUTE_H
#define PICK_EXCUTE_H
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/collision_detection/collision_matrix.h>

#include "msgs/posestamped.h"

#include "hsr_gripper_driver/open_srv.h"

#include <utils/idebug.h>
#include "pickplace/execute_process.h"
#include "pickplace/c_base_pickplace.h"
#include "hpluginloader.h"

#define MOVE_GROUP "arm"

using namespace hirop_pickplace;
using namespace moveit::planning_interface;

class ClassicActuator:public CBasePickPlace{
    typedef struct Geometry{
        std::string type;
        double length;
        double width;
        double height;
        std::string mesh;
        std::string table_link;
        double table_px;
        double table_py;
        double table_pz;
    }Geometry;

    typedef struct MoveitConfig{
        std::string planner_id;
        double velocity;
        double accelerated;
    }MoveitConfig;

    typedef struct PickPlaceConfig
    {
        bool direct;
        std::vector<double> m_Roll;
        std::vector<double> m_Pitch;
        std::vector<double> m_Yaw;

        float pre_dist_min;
        float pre_dist_max;
        double pre_vect_x;
        double pre_vect_y;
        double pre_vect_z;

        float back_dist_min;
        float back_dist_max;
        double back_vect_x;
        double back_vect_y;
        double back_vect_z;
    }PickConfig;

    typedef struct GripperConfig{
        std::string joint_name;
        double open_position;
        double close_position;
    }GripperConfig;

    typedef struct Parameters{
        Geometry geometry;
        MoveitConfig moveitConfig;
        PickPlaceConfig pickConfig;
        PickPlaceConfig placeConfig;
        GripperConfig gripperConfig;
    }Parameters;

public:
    /**
     * @brief 构造函数
     */
    ClassicActuator();

    /**
     * @brief 析构函数
     */
    ~ClassicActuator();

    /**
     * @brief 传递私有参数
     * @return
     * 0,成功
     * -1,失败
     */
    int parseConfig(YAML::Node& );

    /**
     * @brief 设置抓取点位
     * @param pickPos，抓取点位
     * @return
     * 0, 成功
     * -1,失败
     */
    int setPickPose(PoseStamped pickPos);

    /**
     * @brief 设置放置点位
     * @param placePos，　放置点位
     * @return
     * 0, 成功
     * -1,失败
     */
    int setPlacePose(PoseStamped placePos);

    /**
     * @brief 显示物体
     * @param object_pose
     * @return
     */
    int showObject(PoseStamped object_pose);

    /**
     * @brief 删除显示物体
     * @return
     * 0, 成功
     * -1, 失败
     */
    int removeObject();

    /**
     * @brief 运动到点
     * @param pose，点位
     * @return
     */
    int moveToPos(PoseStamped pose);

    /**
     * @brief 运动设置点
     * @param posName，点位名字
     * @return
     */
    int moveToFoName(std::string posName);

    /**
     * @brief 抓取
     * @return
     */
    int pick();

    /**
     * @brief 放置
     * @return
     */
    int place();

    /**
     * @brief 停止抓取放置
     * @return
     */
    int stopPickplace();

private:

    /**
     * @brief _init 参数初始化
     * @return
     */
    void _init();

    /**
     * @brief loadMoveit 加载moveit对象
     * @return
     */
    int loadMoveit();

    /**
     * @brief setMoveitConfig 设置moveit参数
     * @return
     */
    int setMoveitConfig();

    /**
     * @brief setPlannerId 设置moveIt规划器
     * @param plannerId[in] 规划器名
     * @return
     */
    int setPlannerId(std::string plannerId);

    /**
     * @brief setPickplaceSpeed 设置moveit执行最大速度
     * @param velocity　速度
     * @param accelerated　加速度
     * @return
     */
    int setPickplaceSpeed(double velocity,double accelerated);

    /**
     * @brief 四元数转欧拉角
     * @param 输入欧拉角
     * @param 输出四元数
     * @return 0 成功， -1 失败
     */
    int quaternionToEuler(Quaternion quat,euler &pick_euler);

    /**
     * @brief 欧拉角转四元素
     * @param euler_angle，输入欧拉角
     * @param quat ，输出，四元素
     * @return 0
     */
    int eulerToQuaternion(euler euler_angle, Quaternion &quat);

    /**
     * @brief getDiraction 生成抓取放置方向
     * @param pose　点位
     * @param Roll  摇摆值
     * @param Pitch　摇摆值
     * @param Yaw　摇摆值
     * @param vect　方向分量
     * @param quats　四元数
     * @return
     */
    int getDiraction(geometry_msgs::PoseStamped pose, std::vector<double> Roll,
                     std::vector<double> Pitch, std::vector<double> Yaw, pick_vect& vect,
                     std::vector<Quaternion>& quats);

    /**
     * @brief makeMoreQuat 生成摇摆数组
     * @param euler_angle　欧拉角
     * @param Roll　摇摆值
     * @param Pitch　摇摆值
     * @param Yaw　摇摆值
     * @param quats　四元数
     * @return
     */
    int makeMoreQuat(euler euler_angle,std::vector<double> Roll, std::vector<double> Pitch,
                     std::vector<double> Yaw, std::vector<Quaternion>& quats);

    /**
     * @brief makeGrasp　生成抓取参数
     * @return
     */
    int makeGrasp();

    /**
     * @brief makePlace　生成放置参数
     * @return
     */
    int makePlace();

    /**
     * @brief addBaseTable 生成地板
     * @return
     */
    int addBaseTable();

    /**
     * @brief addCollisionObject 配置场景物体
     * @param object_id　物体id
     * @return
     */
    int addCollisionObject(std::string object_id);

    /**
     * @brief addCollision 添加场景物体
     * @return
     */
    int addCollision();

    /**
     * @brief removeCollision 移除场景物体
     * @return
     */
    int removeCollision();

    /**
     * @brief attachCollision　绑定场景物体
     * @param obj　物体ｉｄ
     * @return
     */
    int attachCollision(std::string obj);

    /**
     * @brief dettachCollision 解绑场景物体
     * @param obj　物体id
     * @return
     */
    int dettachCollision(std::string obj);

    /**
     * @brief showObjectMash 加载ｍａｓｈ类型物体
     * @param obj_path　物体路径
     * @return
     */
    int showObjectMash(std::string obj_path);

    /**
     * @brief makeGripperPosture 生成姿态
     * @param jiont_position
     * @return
     */
    trajectory_msgs::JointTrajectory makeGripperPosture(double jiont_position);

    /**
     * @brief gripperOpen 打开夹爪
     * @param speed
     * @return
     */
    int gripperOpen(int speed);

private:
    Parameters m_parm;
    ros::NodeHandle npick;
    geometry_msgs::PoseStamped m_pickPose;
    geometry_msgs::PoseStamped m_placePose;
    std::vector<moveit_msgs::Grasp> _graspPoses;
    std::vector<moveit_msgs::PlaceLocation> _placeLocPoses;
    std::vector<moveit_msgs::CollisionObject> _collisionObjects;
    MoveGroupInterface *_moveGroup;
    PlanningSceneInterface _planScene;
    tf2::Quaternion _orientation;
    std::vector<std::string> _objectIds;
    geometry_msgs::PoseStamped _boxPose;
    geometry_msgs::PoseStamped _transitionPose;
    collision_detection::AllowedCollisionMatrix allowed;
    const std::string _enfLink;
    bool _pickStopFlag;
    const double esp = 1e-3;
};
H_DECLARE_PLUGIN(hirop_pickplace::IPickPlace)
#endif // PICK_EXCUTE_H
