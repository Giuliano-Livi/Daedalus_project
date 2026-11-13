import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_bringup = os.path.join(
        get_package_share_directory('daedalus_trajectories'), 'launch', 'sim_bringup.launch.py')
    slam = os.path.join(
        get_package_share_directory('daedalus_slam'), 'launch', 'slam.launch.py')

    return LaunchDescription([
        # starts the gz bridge (clock, imu, camera) from sim_bringup:
        # set it to false if sim_bringup.launch.py is already running, to avoid a duplicate bridge
        DeclareLaunchArgument('bridge', default_value='true'),
        # RTAB-Map SLAM in parallelo al volo (slam:=true); il bridge resta quello incluso qui sotto
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('odom', default_value='px4'),
        # rumore realistico ZED 2i sulle immagini della camera
        DeclareLaunchArgument('camera_noise', default_value='false'),
        # setpoint corretti con le loop closure di rtabmap (TF map -> odom): utile senza GPS, dove il
        # frame di PX4 deriva; con il GPS non serve
        DeclareLaunchArgument('slam_correction', default_value='false'),
        # odom:=openvins: test senza visione per PX4, "inizio,durata,..." in s dal decollo (vision_to_px4)
        DeclareLaunchArgument('dropouts', default_value=''),
        # odom:=openvins: correzione della prua con la struttura del magazzino (manhattan_yaw)
        DeclareLaunchArgument('manhattan', default_value='true'),
        # odom:=openvins: correzione della quota con il pavimento (manhattan_yaw)
        DeclareLaunchArgument('floor', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={'camera': 'true', 'camera_noise': LaunchConfiguration('camera_noise')}.items(),
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            launch_arguments={
                'bridge': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'odom': LaunchConfiguration('odom'),
                'dropouts': LaunchConfiguration('dropouts'),
                'manhattan': LaunchConfiguration('manhattan'),
                'floor': LaunchConfiguration('floor'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
        Node(
            package='daedalus_trajectories',
            executable='trajectory_generator',
            output='screen'
        ),
        Node(
            package='daedalus_trajectories',
            executable='offboard_control',
            parameters=[{'slam_correction': ParameterValue(LaunchConfiguration('slam_correction'), value_type=bool)}],
            output='screen'
        ),
    ])
