#include "pointcloud_publish.h"

#if PCL_VERSION_COMPARE(>=,1,7,0)
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif

using namespace hirop_perception;

PointCloudPublish::PointCloudPublish(){
    mHandler = ros::NodeHandle();
}

void PointCloudPublish::declare_params(ecto::tendrils &params){

    params.declare<std::string>(&PointCloudPublish::_frameId, "frame_id", \
                                "The point cloud msg frame id").required(true);

}

void PointCloudPublish::declare_io(const ecto::tendrils &params, ecto::tendrils &in, ecto::tendrils &out){

}

void PointCloudPublish::configure(const ecto::tendrils &params, const ecto::tendrils &inputs,
                                  const ecto::tendrils &outputs){
    pointCloudPub = mHandler.advertise<sensor_msgs::PointCloud2>("/test/point", 1);
}

template <typename Point>
int PointCloudPublish::process(const tendrils& inputs, const tendrils& outputs,
                               boost::shared_ptr<const pcl::PointCloud<Point> >& input){

    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);

#if PCL_VERSION_COMPARE(<,1,7,0)
    ::pcl::toROSMsg(*input, *msg);
#else
    pcl::PCLPointCloud2 pcd_tmp;
    pcl::toPCLPointCloud2(*input, pcd_tmp);
    pcl_conversions::fromPCL(pcd_tmp, *msg);
#endif
    msg->header.frame_id = *_frameId;
    pointCloudPub.publish(*msg);

    return ecto::OK;
}

ECTO_CELL(hirop_perception, ecto::pcl::PclCell<PointCloudPublish>,\
          "PointCloudPublish", "Publish point cloud to ros")
