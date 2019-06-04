#include "region_filters.h"

using namespace hirop_perception;

float RegionFilters::minX = -2000;
float RegionFilters::maxX = 2000;
float RegionFilters::minZ = -2000;
float RegionFilters::maxZ = 2000;
float RegionFilters::minY = -2000;
float RegionFilters::maxY = 2000;


RegionFilters::RegionFilters(){

}
RegionFilters::~RegionFilters(){

}

void RegionFilters::declare_params(ecto::tendrils &params){
    params.declare<float>("minZ", "point mini z", -2000);
    params.declare<float>("maxZ", "point max z", 2000);
    params.declare<float>("minX", "point mini x", -2000);
    params.declare<float>("maxX", "point max x", 2000);
    params.declare<float>("minY", "point mini y", -2000);
    params.declare<float>("maxY", "point max y", 2000);
}

void RegionFilters::declare_io(const ecto::tendrils &params, ecto::tendrils &in, ecto::tendrils &out){
    in.declare<ecto::pcl::PointCloud>("input", "The colud to filter");
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
}

void RegionFilters::configure(const ecto::tendrils &params, const ecto::tendrils &inputs, const ecto::tendrils &outputs){
    minZ = params.get<float>("minZ");
    maxZ = params.get<float>("maxZ");
    minX = params.get<float>("minX");
    maxX = params.get<float>("maxX");
    minY = params.get<float>("minY");
    maxY = params.get<float>("maxY");
}

int RegionFilters::process(const ecto::tendrils &in, const ecto::tendrils &out){
    _pointCloud = in.get<ecto::pcl::PointCloud>("input");
    ecto::pcl::xyz_cloud_variant_t variant = _pointCloud.make_variant();

    ecto::pcl::PointCloud outCloud;

    FiltersDispatch dispatch(&outCloud);

    boost::apply_visitor(dispatch, variant);

    out.get<ecto::pcl::PointCloud>("output") = outCloud;

    return ecto::OK;
}

template<typename Point>
void RegionFilters::FiltersDispatch::operator()(boost::shared_ptr<const pcl::PointCloud<Point> >& cloud) const{

    typename pcl::PointCloud<Point>::Ptr outPointCloud(new pcl::PointCloud<Point>);

    int size = cloud->points.size();
    for(int i =0; i < size; i++){
        if(cloud->points[i].z > minZ && cloud->points[i].z < maxZ \
                && cloud->points[i].x > minX && cloud->points[i].x < maxX \
                && cloud->points[i].y > minY && cloud->points[i].y < maxY)
            outPointCloud->points.push_back(cloud->points[i]);
    }

    outPointCloud->width = 1;
    outPointCloud->height = outPointCloud->points.size();
    *_outPointCloud = ecto::pcl::PointCloud(outPointCloud);
}

RegionFilters::FiltersDispatch::FiltersDispatch(ecto::pcl::PointCloud *outCloud){
    _outPointCloud = outCloud;
}


ECTO_CELL(hirop_perception, hirop_perception::RegionFilters, "RegionFilters", \
          "A point cloud region filters")
