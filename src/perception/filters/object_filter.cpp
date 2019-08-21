#include "object_filter.h"

#include <tf/transform_listener.h>

using namespace hirop_perception;

float ObjectFilter::minX = 0;
float ObjectFilter::maxX = 0;
float ObjectFilter::minZ = 0;
float ObjectFilter::maxZ = 0;
float ObjectFilter::minY = 0;
float ObjectFilter::maxY = 0;

ObjectFilter::ObjectFilter(){

    _n = ros::NodeHandle();

    std::cout << "in Object Filter" << std::endl;

}

ObjectFilter::~ObjectFilter(){

}

void ObjectFilter::declare_params(ecto::tendrils &params){
    params.declare<float>("hight", "object hight", 0.1);
    params.declare<float>("width", "object width", 0.1);
    params.declare<float>("length", "object length", 0.1);
}

void ObjectFilter::declare_io(const ecto::tendrils &params, ecto::tendrils &in, ecto::tendrils &out){
    in.declare<ecto::pcl::PointCloud>("input", "The colud to filter");
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
}

void ObjectFilter::configure(const ecto::tendrils &params, const ecto::tendrils &inputs, const ecto::tendrils &outputs){
    hight = params.get<float>("hight");
    width = params.get<float>("width");
    length = params.get<float>("length");
    _objectSub = _n.subscribe("/object_array", 1, &ObjectFilter::objectDetectionCallback, this);
}

int ObjectFilter::process(const ecto::tendrils &in, const ecto::tendrils &out){

    _pointCloud = in.get<ecto::pcl::PointCloud>("input");

    ecto::pcl::xyz_cloud_variant_t variant = _pointCloud.make_variant();

    ecto::pcl::PointCloud outCloud;

    FiltersDispatch dispatch(&outCloud);

    boost::apply_visitor(dispatch, variant);

    out.get<ecto::pcl::PointCloud>("output") = outCloud;

    return ecto::OK;
}


void ObjectFilter::objectDetectionCallback(const vision_bridge::ObjectArray::ConstPtr &msg){
    geometry_msgs::PoseStamped _objectCamPose = msg->objects[0].pose;

    /**
     * @brief _worldPose    save the world pose
     */
    geometry_msgs::PoseStamped worldPose;

    tf::TransformListener listener;

    std::cout << "in objectDetectionCallback" << std::endl;

    for(int i = 0; i < 5; i++){

        try{
            listener.waitForTransform("base_link", _objectCamPose.header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.transformPose("base_link",_objectCamPose, worldPose);
        }catch (tf::TransformException &ex) {
            /**
         * @brief 获取变换关系失败，等待1秒后继续获取坐标变换
         */
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            break;
        }

    }

    minX = worldPose.pose.position.x - width/2;
    minY = worldPose.pose.position.y - length/2;
    minZ = worldPose.pose.position.z - hight/2;

    maxX = worldPose.pose.position.x + width/2;
    maxY = worldPose.pose.position.y + length/2;
    maxZ = worldPose.pose.position.z + hight/2;

}

template<typename Point>
void ObjectFilter::FiltersDispatch::operator()(boost::shared_ptr<const pcl::PointCloud<Point> >& cloud) const{

    typename pcl::PointCloud<Point>::Ptr outPointCloud(new pcl::PointCloud<Point>);

    int size = cloud->points.size();
    for(int i = 0; i < size; i++){
        if(cloud->points[i].z > minZ && cloud->points[i].z < maxZ \
                && cloud->points[i].x > minX && cloud->points[i].x < maxX \
                && cloud->points[i].y > minY && cloud->points[i].y < maxY)
            continue;
        outPointCloud->points.push_back(cloud->points[i]);
    }

    outPointCloud->width = 1;
    outPointCloud->height = outPointCloud->points.size();
    *_outPointCloud = ecto::pcl::PointCloud(outPointCloud);
}

ObjectFilter::FiltersDispatch::FiltersDispatch(ecto::pcl::PointCloud *outCloud){
    _outPointCloud = outCloud;
}

ECTO_CELL(hirop_perception, hirop_perception::ObjectFilter, "ObjectFilter", \
          "A point cloud object filters")
