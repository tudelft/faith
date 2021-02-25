cmake_minimum_required(VERSION 2.8.3)
project(dvs_of)

find_package(catkin_simple REQUIRED
   message_generation
)

catkin_simple(CATKIN_DEPENDS message_runtime)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(dvs_of
  src/calibration.cpp
  src/planefitting.cpp
  src/of_server.cpp
  src/of_server_node.cpp
)

# make the nodelet into a library
cs_add_library(dvs_of_nodelet
  src/of_server_nodelet.cpp
  src/of_server.cpp
  src/planefitting.cpp
  src/calibration.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_of
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(dvs_of_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES dvs_of_nodelet.xml nodelet_mono.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



