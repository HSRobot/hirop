#include "object_filter.h"

#include <tf/transform_listener.h>

using namespace hirop_perception;

ObjectFilter::ObjectFilter(){

    minX = 0;
    minY = 0;
    minZ = 0;

    maxX = 0;
    maxY = 0;
    maxZ = 0;

    _objectSub = _n.subscribe("/object_array", 1, &ObjectFilter::objectDetectionCallback, this);

}

ObjectFilter::~ObjectFilter(){

}

void ObjectFilter::declare_params(ecto::tendrils &params){
    params.declare<float>("radius", "object radius", 0.1);
}

void ObjectFilter::declare_io(const ecto::tendrils &params, ecto::tendrils &in, ecto::tendrils &out){
    in.declare<ecto::pcl::PointCloud>("input", "The colud to filter");
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
}

void ObjectFilter::configure(const ecto::tendrils &params, const ecto::tendrils &inputs, const ecto::tendrils &outputs){
    radius = params.get<float>("radius");
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

    minX = worldPose.pose.position.x - radius;
    minY = worldPose.pose.position.y - radius;
    minZ = worldPose.pose.position.z - radius;

    maxX = worldPose.pose.position.x + radius;
    maxY = worldPose.pose.position.y + radius;
    maxZ = worldPose.pose.position.z + radius;

}

template<typename Point>
void ObjectFilter::FiltersDispatch::operator()(boost::shared_ptr<const pcl::PointCloud<Point> >& cloud) const{

    typename pcl::PointCloud<Point>::Ptr outPointCloud(new pcl::PointCloud<Point>);

    int size = cloud->points.size();
    for(int i = 0; i < size; i++){
        if(cloud->points[i].z < minZ && cloud->points[i].z > maxZ \
                && cloud->points[i].x < minX && cloud->points[i].x > maxX \
                && cloud->points[i].y < minY && cloud->points[i].y > maxY)
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
