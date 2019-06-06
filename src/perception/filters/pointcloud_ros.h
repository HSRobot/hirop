#ifndef __POINTCLOUD_ROS_H__
#define __POINTCLOUD_ROS_H__

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

using ecto::tendrils;

namespace hirop_perception {

/**
 * @brief       通过ROS结果获取点云的输入
 * @author      XuKunLin
 * @date        2019-06-05
 */
class PointCloudRos{

public:

    /**
     * @brief   构造函数
     */
    PointCloudRos();

    /**
     * @brief   析构函数
     */
    ~PointCloudRos();

    /**
     * @brief   定义过滤器参数
     */
    static void declare_params(tendrils& params);

    /**
     * @brief   定义过滤器输入输出
     */
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);

    /**
     * @brief   过滤器的过滤实现
     */
    int process(const tendrils& in, const tendrils& out);

    /**
     * @brief   过滤器的配置
     */
    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs);

    /**
     * @brief       点云话题的监听回调函数
     * @param msg   接收到的点云数据
     */
    void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg);

private:
    /**
     * @brief mHandler  保存ROS节点句柄
     */
    ros::NodeHandle mHandler;

    /**
     * @brief _pointCloud   保存上一次接收到的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloud;

    ros::Subscriber pointCloudSub;

    bool havePointCloud;

};

}

#endif
