cmake_minimum_required(VERSION 3.8)
project(daedalus_exploration)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(daedalus_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# autonomous exploration: takeoff, 360 deg scan, then AEP on the 3D map (ExplorationMap + MapManager)
add_executable(auto_exploration
  src/auto_exploration.cpp
  src/exploration_map.cpp
  src/map_manager.cpp)
target_include_directories(auto_exploration PRIVATE include ${EIGEN3_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(auto_exploration ${OCTOMAP_LIBRARIES})
ament_target_dependencies(auto_exploration rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen rtabmap_msgs daedalus_interfaces)

foreach(target auto_exploration)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS auto_exploration DESTINATION lib/${PROJECT_NAME})
# launch files (Python, as usual in ROS 2), RViz configuration
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
