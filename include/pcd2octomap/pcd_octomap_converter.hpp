#ifndef PCD_OCTOMAP_CONVERTER_HPP
#define PCD_OCTOMAP_CONVERTER_HPP

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

namespace octomap_converter
{
    template <typename PointT>
    class Converter
    {
    public:
        using PointCloud = typename pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        Converter();
        ~Converter();

        void convert();

    private:
        ros::NodeHandle nh_;

        Eigen::Isometry3f map_transform_;

        PointCloudPtr pcl_;
        std::shared_ptr<pcl::VoxelGrid<PointT>> vg_filter_;

        std::unique_ptr<octomap::OcTree> octree_;
        std::unique_ptr<octomap::ColorOcTree> color_octree_;

        std::string package_path_;
        std::string pcd_name_;
        std::string pcd_path_;
        std::string ot_name_;
        std::string ot_path_;
        double octree_resolution_;
        double voxel_grid_leaf_size_;
        struct map_pose
        {
            float roll;
            float pitch;
            float yaw;
        };
        map_pose map_pose_;

        inline void getParameters(ros::NodeHandle &nh);

        inline Eigen::Isometry3f setMapTransform(const map_pose &map_pose);

        inline PointCloudPtr initPointCloud(const std::string &pcd_path, const Eigen::Isometry3f &map_transform);
    };
}

#endif //! PCD_OCTOMAP_CONVERTER_HPP