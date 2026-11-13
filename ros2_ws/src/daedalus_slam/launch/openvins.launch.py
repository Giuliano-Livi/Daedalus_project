import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# OpenVINS (stereo + ZED IMU) -> /vo/odom: included by slam.launch.py (odom:=openvins) and by
# odom_replay.launch.py (estimator:=openvins). Inputs: /camera/color/image_raw, /camera/right/image_raw
# (config/openvins/kalibr_imucam_chain.yaml), /zed/imu/data (kalibr_imu_chain.yaml), TF base_link -> zed_imu_link
def generate_launch_description():
    config = os.path.join(get_package_share_directory('daedalus_slam'), 'config', 'openvins', 'estimator_config.yaml')
    return LaunchDescription([
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            namespace='ov_msckf',
            parameters=[{
                'use_sim_time': True,
                'config_path': config,
                'use_stereo': True,
                'max_cameras': 2,
                # its TF (global -> imu -> cam0/1) are stamped with now(): not needed, /vo/odom is enough
                'publish_global_to_imu_tf': False,
                'publish_calibration_tf': False,
            }],
            output='screen',
        ),
        # pose of the IMU in OpenVINS' frame -> pose of base_link, at the camera rate
        Node(
            package='daedalus_slam',
            executable='openvins_odom',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
