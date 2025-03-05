#ifndef PCD_OCTOMAP_CONVERTER_IMPL_HPP
#define PCD_OCTOMAP_CONVERTER_IMPL_HPP

#include <ros/package.h>
#include <tf/tf.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#include "pcd2octomap/pcd_octomap_converter.hpp"

namespace octomap_converter
{
    template <typename PointT>
    Converter<PointT>::Converter() : nh_("~"),
                                     map_transform_(Eigen::Isometry3f::Identity())
    {
        getParameters(nh_);
        pcl_.reset(new PointCloud);
        vg_filter_ = std::make_shared<pcl::VoxelGrid<PointT>>();
        if constexpr (std::is_same_v<PointT, pcl::PointXYZ>)
        {
            ROS_INFO("\033[32m point cloud type is pcl::PointXYZ \033[0m");
            octree_ = std::make_unique<octomap::OcTree>(octree_resolution_);
        }
        if constexpr (std::is_same_v<PointT, pcl::PointXYZRGB> || std::is_same_v<PointT, pcl::PointXYZRGBA>)
        {
            ROS_INFO("\033[32m point cloud type is pcl::PointXYZRGB or pcl::PointXYZRGBA \033[0m");
            color_octree_ = std::make_unique<octomap::ColorOcTree>(octree_resolution_);
        }
        map_transform_ = setMapTransform(map_pose_);
        const auto pcl_temp_ = initPointCloud(pcd_path_, map_transform_);
        if (pcl_temp_)
        {
            pcl_->swap(*pcl_temp_);
        }
        else
        {
            ROS_ERROR("Point cloud initialization failed!");
        }
    }

    template <typename PointT>
    Converter<PointT>::~Converter() = default;

    template <typename PointT>
    inline void Converter<PointT>::getParameters(ros::NodeHandle &nh)
    {
        nh.param<std::string>("pcd_name", pcd_name_, std::string(""));
        nh.param<std::string>("ot_name", ot_name_, std::string(""));
        nh.param<double>("octree_resolution", octree_resolution_, 0.05);
        nh.param<double>("voxel_grid_leaf_size", voxel_grid_leaf_size_, 0.05);
        nh.param<float>("map_pose/roll", map_pose_.roll, 0.0);
        nh.param<float>("map_pose/pitch", map_pose_.pitch, 0.0);
        nh.param<float>("map_pose/yaw", map_pose_.yaw, 0.0);
        if (octree_resolution_ <= 0.0)
        {
            throw std::invalid_argument("Octree resolution must be positive!");
        }
        package_path_ = ros::package::getPath("pcd2octomap");
        pcd_path_ = package_path_ + "/pcd/" + pcd_name_;
        ot_path_ = package_path_ + "/octomap/" + ot_name_;
    }

    template <typename PointT>
    inline Eigen::Isometry3f Converter<PointT>::setMapTransform(const map_pose &map_pose)
    {
        Eigen::Isometry3f map_transform;
        map_transform = Eigen::Isometry3f::Identity();

        Eigen::AngleAxisf rollAngle(map_pose.roll * M_PI / 180.0f, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(map_pose.pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(map_pose.yaw * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation_matrix = (rollAngle * pitchAngle * yawAngle).matrix();

        map_transform.linear() = rotation_matrix;
        map_transform.translation() = Eigen::Vector3f::Zero();

        return map_transform;
    }

    template <typename PointT>
    inline typename Converter<PointT>::PointCloudPtr Converter<PointT>::initPointCloud(const std::string &pcd_path, const Eigen::Isometry3f &map_transform)
    {
        PointCloudPtr pcl_in(new PointCloud);
        PointCloudPtr pcl_trans(new PointCloud);
        PointCloudPtr pcl_out(new PointCloud);
        if (pcl::io::loadPCDFile(pcd_path_, *pcl_in) == -1)
        {
            ROS_ERROR("Couldn't read pcd file %s", pcd_path_.c_str());
            return nullptr;
        }
        ROS_INFO("\033[32m input point size:\033[0m\033[34m %ld \033[0m", pcl_in->size());
        pcl_trans->reserve(pcl_in->size());
        pcl_out->reserve(pcl_in->size());

        /* eigen to tf */
        tf::Vector3 t(map_transform.translation().x(),
                      map_transform.translation().y(),
                      map_transform.translation().z());
        Eigen::Matrix3f rotation = map_transform.rotation();
        Eigen::Quaternionf q_eigen(rotation);
        tf::Quaternion q(
            q_eigen.x(),
            q_eigen.y(),
            q_eigen.z(),
            q_eigen.w());
        tf::Transform tf_map_transform;
        tf_map_transform.setOrigin(t);
        tf_map_transform.setRotation(q);
        pcl_ros::transformPointCloud(*pcl_in, *pcl_trans, tf_map_transform);

        /* filter */
        vg_filter_->setInputCloud(pcl_trans);
        vg_filter_->setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        vg_filter_->filter(*pcl_out);
        ROS_INFO("\033[32m output point size:\033[0m\033[34m %ld \033[0m", pcl_out->size());
        return pcl_out;
    }

    template <typename PointT>
    inline void Converter<PointT>::convert()
    {
        if constexpr (std::is_same_v<PointT, pcl::PointXYZ>)
        {
            for (const auto &pt : pcl_->points)
            {
                octree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
            }
            octree_->updateInnerOccupancy();
            octree_->writeBinary(ot_path_);
        }
        if constexpr (std::is_same_v<PointT, pcl::PointXYZRGB> || std::is_same_v<PointT, pcl::PointXYZRGBA>)
        {
            for (const auto &pt : pcl_->points)
            {
                color_octree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
            }
            for (const auto &pt : pcl_->points)
            {
                color_octree_->integrateNodeColor(pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);
            }
            color_octree_->updateInnerOccupancy();
            color_octree_->writeBinary(ot_path_);
        }
        ROS_INFO("\033[32m Octomap saved to\033[0m\033[34m %s \033[0m", ot_path_.c_str());
    }
}

#endif //! PCD_OCTOMAP_CONVERTER_IMPL_HPP