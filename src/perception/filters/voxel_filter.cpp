#include "voxel_filter.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

using namespace hirop_perception;

VoxelFilter::VoxelFilter(){

}

void VoxelFilter::configure(const tendrils &params, const tendrils &inputs, const tendrils &outputs){

}

void VoxelFilter::declare_params(tendrils &params){

}

void VoxelFilter::declare_io(const tendrils &params, tendrils &in, tendrils &out){

    out.declare<ecto::pcl::PointCloud>("output", "The colud to out");

}

template<typename Point>
int VoxelFilter::process(const tendrils& inputs, const tendrils& outputs,
                         boost::shared_ptr<const pcl::PointCloud<Point> >& input){

    typename pcl::PointCloud<Point>::Ptr cloudFiltered(new pcl::PointCloud<Point>());

    std::cout << "PointCloud before filtering: " << input->width * input->height << std::endl;

    pcl::VoxelGrid< typename pcl::PointCloud<Point> > sor;
    sor.setInputCloud (input);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloudFiltered);

    std::cout << "PointCloud after filtering: " << cloudFiltered->width * cloudFiltered->height << std::endl;

    outputs.get<ecto::pcl::PointCloud>("output") = ecto::pcl::PointCloud(cloudFiltered);

    return ecto::OK;
}

