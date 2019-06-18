#ifndef __PCL_FUSION_H__
#define __PCL_FUSION_H__

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <Eigen/Dense>

using ecto::tendrils;

namespace hirop_perception {

/**
 * @brief       三维点云融合类，通过两个点云之间的变换矩阵将两个点云进行融合
 *              融合点云需要点云、及当前相机坐标和世界坐标的变换关系。
 *              ECTO相关：
 *                  参数clean: 清空缓存的点云
 *                  输入input: 点云
 *                  输入R:     旋转矩阵
 *                  输入T:     平移矩阵
 *                  输出output: 融合后的点云
 * @author      XuKunLin
 * @date        2019-06-03
 */
class PclFusion{

public:
    /**
     * @brief   构造函数
     */
    PclFusion();

    /**
     * @brief   析构函数
     */
    ~PclFusion();

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

    /**
     * @brief testFlagChange    当修改测试标志时的回调函数
     * @param falg              修改后的标志
     */
    void testFlagChange(bool falg);

private:

    /**
     * @brief _pointCloud   保存生成的点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloud;

    /**
     * @brief _testFlag     测试标志位
     */
    bool _testFlag;

    /**
     * @brief haveCache     当前是否已经融合过点云的标志位
     */
    bool haveCache;

    /**
     * @brief isSaving      当前循环处于保存文件的状态
     */
    ecto::spore<bool> isSaving;

    /**
     * @brief isCleaning    当前循环清空已缓存的点云
     */
    ecto::spore<bool> isCleaning;

    /**
     * @brief _outPointCloud    保存已融合的点云
     */
    ecto::pcl::PointCloud _outPointCloud;
};
}

#endif
