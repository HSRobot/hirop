#ifndef __REGION_FILTERS_H__
#define __REGION_FILTERS_H__

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

ECTO_DEFINE_MODULE(hirop_perception) { }

using ecto::tendrils;

namespace hirop_perception {

/**
 * @brief       点云区域过滤器
 * @author      XuKunLin
 * @date        2019-05-30
 */
class RegionFilters{

public:

    /**
     * @brief   构造函数
     */
    RegionFilters();

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
     * @brief   析构函数
     */
    ~RegionFilters();

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
     * @brief _pointCloud   输入的点云
     */
    ecto::pcl::PointCloud _pointCloud;

    /**
     * 关注区域的XYZ最大最小值
     */
    static float minX;
    static float maxX;
    static float minY;
    static float maxY;
    static float minZ;
    static float maxZ;
};

}

#endif
