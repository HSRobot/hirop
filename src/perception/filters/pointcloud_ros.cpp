#include "pointcloud_ros.h"

#define DEBUG
#include "utils/idebug.h"

using namespace hirop_perception;

PointCloudRos::PointCloudRos(){
    mHandler = ros::NodeHandle();
    havePointCloud = false;
}

PointCloudRos::~PointCloudRos(){

}

void PointCloudRos::declare_params(tendrils& params){
    params.declare<std::string>("topic_name", "Which topic will be listen", "");
}

void PointCloudRos::declare_io(const tendrils& params, tendrils& in, tendrils& out){
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
}

void PointCloudRos::configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs){

    IDebug("configure");
    std::string topicName;
    topicName = params.get<std::string>("topic_name");

    pointCloudSub = mHandler.subscribe<sensor_msgs::PointCloud2>(topicName, 1, \
                                                 &PointCloudRos::pointCloudCallBack, this);
}

int PointCloudRos::process(const tendrils& in, const tendrils& out){
    if(!havePointCloud)
        return ecto::DO_OVER;
    out.get<ecto::pcl::PointCloud>("output") = ecto::pcl::PointCloud(_pointCloud);
    return ecto::OK;
}

void PointCloudRos::pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    const sensor_msgs::PointCloud2 tmp = *msg;
    /**
     * 此处会进行深度拷贝
     */
    pcl::fromROSMsg(tmp, *cloud);

    _pointCloud = cloud;

    havePointCloud = true;
}

ECTO_CELL(hirop_perception, PointCloudRos, "PointCloudRos", \
          "A  point cloud source filters by ros topic")
