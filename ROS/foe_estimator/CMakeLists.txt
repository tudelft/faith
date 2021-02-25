cmake_minimum_required(VERSION 3.0.2)
project(foe_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dvs_of
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES foe_estimator
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(foe_estimator 
   src/foe_estimator.cpp
   src/det.cpp
   src/eml_rand_mt19937ar_stateful.cpp
   src/estimateFoECPP.cpp
   src/estimateFoECPP_data.cpp
   src/estimateFoECPP_initialize.cpp
   src/estimateFoECPP_terminate.cpp
   src/find.cpp
   src/minOrMax.cpp
   src/mldivide.cpp
   src/rand.cpp
   src/rtGetInf.cpp
   src/rtGetNaN.cpp
   src/rt_nonfinite.cpp
   src/sign.cpp)

target_link_libraries(foe_estimator ${catkin_LIBRARIES})


