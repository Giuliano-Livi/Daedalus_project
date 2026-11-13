cmake_minimum_required(VERSION 3.8)
project(daedalus_camera)

# the noise loops run on ~1M pixels per frame: an unoptimised build would miss the 15 Hz
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_noise src/camera_noise.cpp)
target_compile_features(camera_noise PUBLIC cxx_std_17)
target_compile_options(camera_noise PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(camera_noise rclcpp sensor_msgs)

# right image of the stereo pair from the left colour + depth (no third rendered camera)
add_executable(stereo_synth src/stereo_synth.cpp)
target_compile_features(stereo_synth PUBLIC cxx_std_17)
target_compile_options(stereo_synth PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(stereo_synth rclcpp sensor_msgs)

install(TARGETS camera_noise stereo_synth DESTINATION lib/${PROJECT_NAME})

ament_package()
