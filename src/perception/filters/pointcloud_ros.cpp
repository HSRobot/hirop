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
    params.declare<std::string>("world_frame", "The world frame name", "");
}

void PointCloudRos::declare_io(const tendrils& params, tendrils& in, tendrils& out){
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
    out.declare<Eigen::Vector3d>("T", "The camera frame translation");
    out.declare<Eigen::Matrix3d>("R", "The camera frame rotation");
}

void PointCloudRos::configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs){

    IDebug("configure");
    std::string topicName;

    topicName = params.get<std::string>("topic_name");
    worldFrame = params.get<std::string>("world_frame");

    pointCloudSub = mHandler.subscribe<sensor_msgs::PointCloud2>(topicName, 1, \
                                                                 &PointCloudRos::pointCloudCallBack, this);
}

int PointCloudRos::process(const tendrils& in, const tendrils& out){

    /**
     *  还未接收到点云，重复执行当前过滤器
     */
    if(!havePointCloud)
        return ecto::DO_OVER;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    /**
     * 获取当前相机系和世界坐标系的变换关系
     */
    try{
        listener.waitForTransform(worldFrame, cameraFrame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(worldFrame, cameraFrame, ros::Time(0), transform);
    }catch (tf::TransformException &ex) {
        /**
         * @brief 获取变换关系失败，等待1秒后继续获取坐标变换
         */
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return ecto::DO_OVER;
    }

    IDebug("transform.x = %f, y = %f, z = %f", transform.getOrigin().x(), \
           transform.getOrigin().y(), transform.getOrigin().z());

    Eigen::Vector3d T;
    Eigen::Matrix3d R;

    rosTf2Eigen(transform, T, R);

    /**
     * 将最后一次收到的点云，发送出去
     */
    out.get<ecto::pcl::PointCloud>("output") = ecto::pcl::PointCloud(_pointCloud);
    out.get<Eigen::Vector3d>("T") = T;
    out.get<Eigen::Matrix3d>("R") = R;
    return ecto::OK;
}

void PointCloudRos::pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg){

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *cloud);

    cameraFrame = msg->header.frame_id;

    /**
     * @brief pcl::fromPCLPointCloud2   将PointCloud2 转换为 PointCloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud, *pcl_cloud);

    _pointCloud = pcl_cloud;

    delete cloud;

    havePointCloud = true;
}

void PointCloudRos::rosTf2Eigen(const tf::StampedTransform &transform, Eigen::Vector3d &t, Eigen::Matrix3d &r){
    t(0) = transform.getOrigin().x();
    t(1) = transform.getOrigin().y();
    t(2) = transform.getOrigin().z();


    tf::Quaternion tmp;
    tmp = transform.getRotation();

    IDebug("TF::Quaternion.x = %f, y = %f, z = %f, w = %f", tmp.x(), tmp.y(), tmp.z(), tmp.w());
    Eigen::Quaterniond q( tmp.w(), tmp.x(), tmp.y(), tmp.z());

    IDebug("Eigen::Quaternion.x = %f, y = %f, z = %f, w = %f", q.x(), q.y(), q.z(), q.w());

    r = q.toRotationMatrix();
}

ECTO_CELL(hirop_perception, PointCloudRos, "PointCloudRos", \
          "A  point cloud source filters by ros topic")
