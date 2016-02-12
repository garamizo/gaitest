cmake_minimum_required(VERSION 2.8.3)
project(gaitest)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS nodelet)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gaitest
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gest src/gest.cpp)
target_link_libraries(gest ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gest ground_est_cpp)

add_executable(gest2 src/gest2.cpp)
target_link_libraries(gest2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gest2 ground_est_cpp)

