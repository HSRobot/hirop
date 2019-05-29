#ifndef __REGION_FILTERS_H__
#define __REGION_FILTERS_H__

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

ECTO_DEFINE_MODULE(hirop_perception) { }

using ecto::tendrils;

namespace hirop_perception {

class RegionFilters{

public:

    RegionFilters();

    static void declare_params(tendrils& params);

    static void declare_io(const tendrils& params, tendrils& in, tendrils& out);

    int process(const tendrils& in, const tendrils& out);

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs);

    ~RegionFilters();

    class FiltersDispatch : public boost::static_visitor<void>{
        void
        operator()(boost::shared_ptr<const CloudPOINTXYZRGB>& cloud) const
        {
            ::pcl::visualization::PointCloudColorHandlerRGBField<CloudPOINTXYZRGB::PointType> rgb(cloud);
            if (!viewer->updatePointCloud(cloud, rgb, key))
            {
                viewer->addPointCloud(cloud, rgb, key);
            }
        }
    };

private:
    ecto::pcl::PointCloud _pointCloud;

};

}

#endif
