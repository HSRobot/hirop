#include "pcl_fusion.h"
#define DEBUG
#include "utils/idebug.h"
using namespace hirop_perception;

PclFusion::PclFusion(){
    _pointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    _testFlag = false;
    haveCache = false;

    IDebug("PCL Funsion create");
}

PclFusion::~PclFusion(){
    _pointCloud.reset();
    IDebug("PCL Funsion detele");
}

void PclFusion::declare_params(tendrils& params){
    params.declare<bool>("clean","clean the cache point cloud");
#ifdef DEBUG
    params.declare<bool>("test", "the debug flag");
#endif
}

void PclFusion::testFlagChange(bool flag){
    _testFlag = flag;
}

void PclFusion::declare_io(const tendrils& params, tendrils& in, tendrils& out){
    /**
     *      当没有获取到坐标变换时，认为世界坐标和相机坐标重合
     */
    Eigen::Matrix3d defaultR;
    defaultR << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Vector3d defaultT(0, 0, 0);

    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
    in.declare<Eigen::Matrix3d>("R", "The rotation Vector", defaultR);
    in.declare<Eigen::Vector3d>("T", "The translation Vector", defaultT);
}

void PclFusion::configure(const tendrils &params, const tendrils &inputs, const tendrils &outputs){
#ifdef DEBUG
    params["test"]->set_callback<bool>(boost::bind(&PclFusion::testFlagChange, this, _1));
#endif
}

template <typename Point>
int PclFusion::process(const tendrils& inputs, const tendrils& outputs,
                       boost::shared_ptr<const pcl::PointCloud<Point> >& input){

#ifdef DEBUG
    /**
     * @brief R_ 测试中，将点云绕Z轴旋转90度
     */
    Eigen::Matrix3d R_;
    if(_testFlag)
        R_ <<   0, 0, 1,
                0, 1, 0,
                -1, 0, 0;
    else
        R_ <<   1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    Eigen::Vector3d T_(0, 0, 0);
#else
    Eigen::Matrix3d R_ = inputs.get<Eigen::Matrix3d>("R");
    Eigen::Vector3d T_ = inputs.get<Eigen::Vector3d>("T");
#endif

    Eigen::Vector3d tmpPos;
    Eigen::Vector3d tmpValue;
    Point tmpPoint;

    /**
     * @brief tmp   该变量的主要目的是作为载体，使得可以修改const变量
     */
    typename pcl::PointCloud<Point>::Ptr tmp(new pcl::PointCloud<Point>);

    /**
     * 如果当前已经融合了部分点云，则无需重新对_outPointCloud进行赋值
     */
    if(!haveCache){
        _outPointCloud = ecto::pcl::PointCloud(tmp);
        haveCache = true;
    }else{
        /**
         * 将const变量通过该方式传递给非const变量
         * 需要注意的是，当经过这个操作后若是直接退出函数，那么在variant中的保存的变量副本会被直接回收掉。
         */
        *tmp =  *(boost::get<typename pcl::PointCloud<Point>::ConstPtr>(_outPointCloud.make_variant()));
    }

    int size = input->points.size();
    for(int i = 0; i < size; i++){
        /**
         *      先计算旋转 [x,y,z] * R
         */
        tmpPos << input->points[i].x, input->points[i].y, input->points[i].z;
        tmpValue = R_ * tmpPos;

        /**
         *      计算平移
         */
        tmpPoint.x = tmpValue(0) + T_(0);
        tmpPoint.y = tmpValue(1) + T_(1);
        tmpPoint.z = tmpValue(2) + T_(2);
        tmp->points.push_back(tmpPoint);
    }

    // 将点云发布出去
    outputs.get<ecto::pcl::PointCloud>("output") = _outPointCloud;
    return ecto::OK;
}

ECTO_CELL(hirop_perception, ecto::pcl::PclCell<PclFusion>,\
          "PclFusion", "Fusion point cloud by R&T")