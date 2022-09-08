cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_process)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  octomap_msgs
  nav_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OCTOMAP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_process
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
DEPENDS
octomap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(pc_pub  src/pc_pub.cpp)
target_link_libraries(pc_pub ${catkin_LIBRARIES} ${PCL_LIBS})

add_executable(pc_pub1  src/pc_pub1.cpp)
target_link_libraries(pc_pub1 ${catkin_LIBRARIES} ${PCL_LIBS})

add_executable(pc_pub_octo  src/pc_pub_octo.cpp)
target_link_libraries(pc_pub_octo ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES})

add_executable(pc_segment  src/pc_segment.cpp)
target_link_libraries(pc_segment ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(pc_segment2  src/pc_segment2.cpp)
target_link_libraries(pc_segment2 ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(pc_intensityadd  src/pc_intensityadd.cpp)
target_link_libraries(pc_intensityadd ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(multi_pc_pub  src/multi_pc_pub.cpp)
target_link_libraries(multi_pc_pub ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(pc_sub  src/pc_sub.cpp)
target_link_libraries(pc_sub ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(pc_aabb src/pc_aabb_generate.cpp)
target_link_libraries(pc_aabb ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(totalpc_pub src/totalpc_pub.cpp)
target_link_libraries(totalpc_pub ${catkin_LIBRARIES} ${PCL_LIBS} ${OCTOMAP_LIBRARIES} )

catkin_install_python(PROGRAMS script/talker  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})