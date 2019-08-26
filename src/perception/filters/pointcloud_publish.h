#ifndef __POINTCLOUD_PUBLISH_H__
#define __POINTCLOUD_PUBLISH_H__

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

using ecto::tendrils;

namespace hirop_perception {

class PointCloudPublish{

public:
    PointCloudPublish();

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
    template <typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const pcl::PointCloud<Point> >& input);

    /**
     * @brief   过滤器的配置
     */
    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs);

private:
    /**
     * @brief mHandler  保存ROS节点句柄
     */
    ros::NodeHandle mHandler;

    ros::Publisher pointCloudPub;

    /**
     * @brief _frameId  保存点云的参考坐标系ID
     */
    ecto::spore<std::string> _frameId;

    /**
     * @brief _topicName  保存点云topic name
     */
    ecto::spore<std::string> _topicName;


};

}

#endif
