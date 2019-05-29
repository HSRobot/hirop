#include "region_filters.h"

using namespace hirop_perception;

RegionFilters::RegionFilters(){

}

void RegionFilters::declare_params(ecto::tendrils &params){
    params.declare<std::string>("prefix", "A string to prefix printing with.", "start>> ");
}

void RegionFilters::declare_io(const ecto::tendrils &params, ecto::tendrils &in, ecto::tendrils &out){
    in.declare<ecto::pcl::PointCloud>("input", "The colud to filter");
    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");
}

void RegionFilters::configure(const ecto::tendrils &params, const ecto::tendrils &inputs, const ecto::tendrils &outputs){

}

int RegionFilters::process(const ecto::tendrils &in, const ecto::tendrils &out){
    _pointCloud = in.get<ecto::pcl::PointCloud>("input");
    ecto::pcl::xyz_cloud_variant_t variant = _pointCloud.make_variant();
}

ECTO_CELL(hirop_perception, hirop_perception::RegionFilters, "RegionFilters", \
          "A point cloud region filters")
