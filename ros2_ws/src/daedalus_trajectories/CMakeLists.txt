cmake_minimum_required(VERSION 3.8)
project(daedalus_trajectories)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(daedalus_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# /drone/setpoint (ENU) -> PX4 offboard (NED), hold, mode switch and arming
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs tf2 tf2_ros daedalus_interfaces)

# A* on the known map + smooth trajectory through the target points
add_executable(trajectory_generator src/trajectory_generator.cpp)
target_include_directories(trajectory_generator PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(trajectory_generator rclcpp ament_index_cpp daedalus_interfaces)

foreach(target offboard_control trajectory_generator)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS offboard_control trajectory_generator DESTINATION lib/${PROJECT_NAME})
# launch files (Python, as usual in ROS 2), RViz configuration, map of the warehouse
install(DIRECTORY launch rviz maps DESTINATION share/${PROJECT_NAME})

ament_package()
