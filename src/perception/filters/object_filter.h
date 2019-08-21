#pragma once

#include <ros/ros.h>

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

#include <vision_bridge/detection.h>
#include <vision_bridge/ObjectArray.h>

using ecto::tendrils;

namespace hirop_perception{

/**
 * @brief The ObjectFilter class    将物体从点云中分割出去的过滤器
 */
class ObjectFilter{

public:

    /**
     * @brief ObjectFilter      构造函数
     */
    ObjectFilter();

    /**
     *  析构函数
     */
    ~ObjectFilter();

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
     * @brief 多种PCL点云类型varinat的访问器
     */
    class FiltersDispatch : public boost::static_visitor<void>{
    public:

        /**
         * @brief FiltersDispatch   构造函数
         * @param outCloud          经过过滤后的点云
         */
        FiltersDispatch(ecto::pcl::PointCloud *outCloud);

        /**
         * @brief varinat 访问器实现
         */
        template<typename Point>
        void operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud) const;

    private:
        /**
         * @brief _outPointCloud    保存经过过滤后的点云
         */
        ecto::pcl::PointCloud *_outPointCloud;
    };

private:

    /**
     * @brief objectDetectionCallback   当有物体被识别到时的回调函数
     * @param msg                       消息
     */
    void objectDetectionCallback(const vision_bridge::ObjectArray::ConstPtr& msg);

private:

    /**
     * @brief _pointCloud   输入的点云
     */
    ecto::pcl::PointCloud _pointCloud;

    /**
     * @brief _n            节点句柄
     */
    ros::NodeHandle _n;

    /**
     * @brief _objectSub    物体检测成功的话题
     */
    ros::Subscriber _objectSub;

    /**
     *
     */
    float hight;
    float width;
    float length;

    /**
     * 物体区域的XYZ最大最小值
     */
    static float minX;
    static float maxX;
    static float minY;
    static float maxY;
    static float minZ;
    static float maxZ;

};

}
