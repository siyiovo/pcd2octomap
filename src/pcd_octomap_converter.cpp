#include "pcd2octomap/pcd_octomap_converter.hpp"
#include "pcd2octomap/impl/pcd_octomap_converter_impl.hpp"
#include "pcd2octomap/point_type_pcd_octomap_converter.hpp"

using namespace octomap_converter;

template class Converter<pcl::PointXYZ>;
template class Converter<pcl::PointXYZRGB>;
template class Converter<pcl::PointXYZRGBA>;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pcd_octomap_converter_node");
    Converter<pcl::PointXYZ> XYZConverter;
    XYZConverter.convert();
    ros::spinOnce();
    return 0;
}
