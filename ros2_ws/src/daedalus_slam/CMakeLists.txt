cmake_minimum_required(VERSION 3.8)
project(daedalus_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(daedalus_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# PX4 odometry -> /odom + TF odom -> base_link
add_executable(px4_odometry src/px4_odometry.cpp)
target_include_directories(px4_odometry PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(px4_odometry rclcpp nav_msgs geometry_msgs px4_msgs tf2_ros)

# visual odometry -> PX4 external vision
add_executable(vision_to_px4 src/vision_to_px4.cpp)
target_include_directories(vision_to_px4 PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(vision_to_px4 rclcpp nav_msgs px4_msgs geometry_msgs daedalus_interfaces)

# OpenVINS odometry (ZED IMU pose) -> /vo/odom (base_link pose), as rgbd_odometry
add_executable(openvins_odom src/openvins_odom.cpp)
target_include_directories(openvins_odom PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(openvins_odom rclcpp nav_msgs tf2_ros)

# heading relative to the warehouse's orthogonal structure (Manhattan world), from the depth image
find_package(sensor_msgs REQUIRED)
add_executable(manhattan_yaw src/manhattan_yaw.cpp)
target_include_directories(manhattan_yaw PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(manhattan_yaw rclcpp nav_msgs sensor_msgs tf2_ros daedalus_interfaces)

foreach(target px4_odometry vision_to_px4 openvins_odom manhattan_yaw)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS px4_odometry vision_to_px4 openvins_odom manhattan_yaw DESTINATION lib/${PROJECT_NAME})
# launch files (Python, as usual in ROS 2), RTAB-Map parameters, RViz configuration
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
