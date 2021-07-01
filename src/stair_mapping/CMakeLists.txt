cmake_minimum_required(VERSION 3.12)
project(stair_mapping)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE RELEASE)
set(Open3D_ROOT /opt/open3d_install)
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

set(TERRAIN_3D_MAP_SOURCE src/terrain_3d_map_node.cpp src/Terrain3dMapperNode.cpp src/Terrain3dMapper.cpp src/SubMap.cpp)
set(TERRAIN_3D_MAP_SOURCE ${TERRAIN_3D_MAP_SOURCE} src/GlobalMap.cpp src/PreProcessor.cpp src/PoseGraph.cpp src/RobotTf.cpp)
add_executable(terrain_3d_map_node ${TERRAIN_3D_MAP_SOURCE})

set(HEIGHT_MAP_SOURCE src/height_map_node.cpp src/ElevationGrid.cpp src/HeightMapNode.cpp)
add_executable(height_map_node ${HEIGHT_MAP_SOURCE})

## Specify libraries to link a library or executable target against
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

