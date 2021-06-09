cmake_minimum_required(VERSION 3.0.2)
project(target_tracker)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -mtune=corei7 -fopenmp -g ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslib
)

find_package(Boost 1.75.0)
find_package(Eigen3)

find_package(eigen-quadprog)

catkin_package(
)

include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

add_library(static_target_2_obs src/2_obs_static_target/fov_2_obs.cpp)
add_library(static_target_3_obs src/3_obs_static_target/fov_3_obs.cpp)
add_library(target_tracker_3_obs src/tracker_dynamic_target/fov_tracker_3_obs.cpp)
add_library(target_tracker_6_obs src/tracker_dynamic_target/6_obs/fov_tracker_6_obs.cpp)


add_executable(mpc_fov_2_obs_ros_node src/2_obs_static_target/mpc_fov_2_obs_ros_node.cpp)
add_dependencies(mpc_fov_2_obs_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_fov_2_obs_ros_node static_target_2_obs eigen-quadprog yaml-cpp ${catkin_LIBRARIES})

add_executable(mpc_fov_3_obs_ros_node src/3_obs_static_target/mpc_fov_3_obs_ros_node.cpp)
add_dependencies(mpc_fov_3_obs_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_fov_3_obs_ros_node static_target_3_obs eigen-quadprog yaml-cpp ${catkin_LIBRARIES})

##Tracker ROS

add_executable(mpc_tracker_3_obs_ros_node src/tracker_dynamic_target/mpc_tracker_3_obs_ros_node.cpp)
add_dependencies(mpc_tracker_3_obs_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker_3_obs_ros_node target_tracker_3_obs eigen-quadprog yaml-cpp ${catkin_LIBRARIES})

add_executable(mpc_tracker_3_obs_dyn_ros_node src/tracker_dynamic_target/mpc_tracker_3_obs_dyn_ros_node.cpp)
add_dependencies(mpc_tracker_3_obs_dyn_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker_3_obs_dyn_ros_node target_tracker_3_obs eigen-quadprog yaml-cpp ${catkin_LIBRARIES})


add_executable(mpc_tracker_6_obs_ros_node src/tracker_dynamic_target/6_obs/mpc_tracker_6_obs_ros_node.cpp)
add_dependencies(mpc_tracker_6_obs_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker_6_obs_ros_node target_tracker_6_obs eigen-quadprog yaml-cpp ${catkin_LIBRARIES})

##


