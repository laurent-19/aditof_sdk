
# Launch file examples

## rviz_publisher
Basic vizualization of point clouds in rviz.
Published: * aditof_pcloud : pcl data
           * tf : base_link (robot frame) -> pointcloud (camera frame)
## camera_node
Basic example of data publishers for all camera data.
Published: * aditof_roscpp/aditof_pcloud : pcl data
           * aditof_roscpp/aditof_depth : 2D depth map
           * aditof_roscpp/aditof_ir : 2D infrared image
           * aditof_roscpp/camera_info : camera details, calibration

## laser_scan_camera
Usage of depth_to_laserscan ROS package for further mapping, navigation.
Published: * aditof_roscpp/aditof_pcloud : pcl data
           * aditof_roscpp/aditof_depth : 2D depth map
           * aditof_roscpp/aditof_ir : 2D infrared image
           * aditof_roscpp/camera_info : camera details, calibration
           * scan : LaserScan (2D laser ranger), base_laser_link frame
           * tf : base_link (robot frame) -> pointcloud (camera frame)
                  base_link (robot frame) -> base_laser_link (laser frame)

- **Install the [depth_to_laserscan](http://wiki.ros.org/depthimage_to_laserscan) package for processing the depth data**
``` 
sudo apt install ros-<distro>-depth-to-laserscan
```

 