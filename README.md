# Livox Lidar PointCloud2

This ROS1 (Noetic) package reads, modifies, and publishes Livox Lidar PointCloud2.

The code is referred from the ROS2 version [livox_to_pointcloud2](https://github.com/porizou/livox_to_pointcloud2).

## How to use?
Modify the ROS1 bag path variable in the file ```src/main.cpp``` (lines 61 and 67), build and run
```
rosrun livox_pointcloud2 livox_pointcloud2_node
```
