cmake_minimum_required(VERSION 3.0.2)
project(livox_pointcloud2)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
