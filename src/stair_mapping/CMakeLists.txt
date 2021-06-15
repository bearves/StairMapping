cmake_minimum_required(VERSION 3.12)
project(stair_mapping)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE RELEASE)
set(Open3D_ROOT /opt/open3d_install)
set(VERBOSE_INFO OFF)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  tf2
  tf2_ros
  roslib
  open3d_conversions
  message_generation
  mini_bridge
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Open3D REQUIRED)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs tf2 tf2_ros roslib open3d_conversions
DEPENDS Open3D
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
  ${CERES_LINK_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
if (${VERBOSE_INFO})
  add_definitions(-DVERBOSE_INFO)
else()
  remove_definitions(-DVERBOSE_INFO)
endif()

add_executable(img_process_node src/img_process_node.cpp src/ImageProcessor.cpp)
add_executable(clip_pcl_data_node src/clip_pcl_data_node.cpp)
add_executable(robot_tf_broadcast_node src/robot_tf_broadcast_node.cpp src/RobotTfBroadcastNode.cpp)
add_executable(test_pose_graph src/test_pose_graph.cpp src/PoseGraph.cpp)

set(TERRAIN_3D_MAP_SOURCE src/terrain_3d_map_node.cpp src/Terrain3dMapperNode.cpp src/Terrain3dMapper.cpp src/SubMap.cpp)
set(TERRAIN_3D_MAP_SOURCE ${TERRAIN_3D_MAP_SOURCE} src/GlobalMap.cpp src/PreProcessor.cpp src/PoseGraph.cpp src/RobotTf.cpp)
add_executable(terrain_3d_map_node ${TERRAIN_3D_MAP_SOURCE})

set(HEIGHT_MAP_SOURCE src/height_map_node.cpp src/ElevationGrid.cpp src/HeightMapNode.cpp)
add_executable(height_map_node ${HEIGHT_MAP_SOURCE})

## Specify libraries to link a library or executable target against
target_link_libraries(img_process_node
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(terrain_3d_map_node PRIVATE Open3D::Open3D
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(height_map_node PRIVATE Open3D::Open3D
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

target_link_libraries(test_pose_graph
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Open3D_LIBRARIES}
)

target_link_libraries(clip_pcl_data_node PRIVATE Open3D::Open3D
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

target_link_libraries(robot_tf_broadcast_node PRIVATE Open3D::Open3D
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
