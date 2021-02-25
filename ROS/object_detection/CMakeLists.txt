cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dvs_of
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detection
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection 
   src/object_detection.cpp
   src/median.cpp
   src/prepEstimateClustersCPP.cpp
   src/prepEstimateClustersCPP_data.cpp
   src/prepEstimateClustersCPP_initialize.cpp
   src/prepEstimateClustersCPP_terminate.cpp
   src/quickselect.cpp
   src/rtGetInf.cpp
   src/rtGetNaN.cpp
   src/rt_nonfinite.cpp
   src/std.cpp
   src/kdtree.cpp
   src/minOrMax.cpp
   src/postEstimateClustersCPP.cpp
   src/postEstimateClustersCPP_data.cpp
   src/postEstimateClustersCPP_initialize.cpp
   src/postEstimateClustersCPP_terminate.cpp
   src/avoidanceCommand.cpp
   src/avoidanceCommand_data.cpp
   src/avoidanceCommand_initialize.cpp
   src/avoidanceCommand_terminate.cpp
   src/controller.cpp
)

target_link_libraries(object_detection ${catkin_LIBRARIES})

