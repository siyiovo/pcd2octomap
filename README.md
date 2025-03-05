# pcd2octomap
package for pcd file convert to octomap file like .bt or .ot.

### Dependencies

1. Eigen >= 3.3.7

2. PCL >= 1.9.0

3. octomap and octovis

   ```
   octomap: sudo apt install ros-noetic-octomap*
   octovis: sudo apt install ros-noetic-octovis*
   ```

   

### Usage

1. `git clone https://github.com/siyiovo/pcd2octomap.git ` to in `$YOUR_WORKSPACE_FOLDER/src`
2. set definition of point type in `/include/pcd2octomap/point_type_pcd_octomap_converter.hpp`

this package support `pcl::PointXYZ`,  `pcl::PointXYZRGB` and `pcl::PointXYZRGBA`.  default : `typedef pcl::PointXYZ PointT;`

3. ```
   catkin build
   source ./devel/setup.bash
   roslaunch pcd2octomap convert.launch
   ```

4. if terminal show `Octomap save to xxx`, just check via octovis

   `octovis $YOUR_OCTOMAP_FILE`



### Parameters

check `config/pcd2octomap_params.yaml`

`octree_resolution`: resolution of octree. default: `0.05`

`voxel_grid_leaf_size`: leaf size of voxel grid filter. default: `0.05`

`map_pose`: control your map rotation. ex. your LiDAR sensor is inverted, you should set `roll: 180` 
