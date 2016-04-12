cmake_minimum_required(VERSION 2.8.3)
project(ekf_localization)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

link_directories(${BFL_LIBRARY_DIRS})


find_package(PCL)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
pcl_ros)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
#   CATKIN_DEPENDS message_runtime
)

include_directories(${BFL_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


# catkin_add_nosetests(test)
add_library(pose_estimation_ekf src/nonlinearanalyticconditionalgaussianmobile.cpp)

add_executable(odom_node src/odom.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ekf_localization_node src/bayesian_filtering/test/BayesianFilterNode.cpp)
target_link_libraries(ekf_localization_node pose_estimation_ekf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
add_dependencies(ekf_localization_node ekf_localization_node_gencpp)

add_executable(ekf_localization_test src/main.cpp src/kalman.cpp src/featuresextractor.cpp)
target_link_libraries(ekf_localization_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
