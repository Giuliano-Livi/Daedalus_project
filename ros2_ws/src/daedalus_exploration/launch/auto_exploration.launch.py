import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_bringup = os.path.join(
        get_package_share_directory('daedalus_trajectories'), 'launch', 'sim_bringup.launch.py')
    slam = os.path.join(
        get_package_share_directory('daedalus_slam'), 'launch', 'slam.launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('daedalus_exploration'), 'rviz', 'exploration.rviz')

    # same pipeline as fly_vio.sh (trajectory_following.launch.py with slam:=true odom:=openvins
    # slam_correction:=true), but the setpoints come from auto_exploration instead of trajectory_generator
    # (both publish /drone/setpoint: never together)
    return LaunchDescription([
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        # RViz2 with the exploration map (rviz/exploration.rviz)
        DeclareLaunchArgument('rviz', default_value='true'),
        # 'openvins' without GPS nor compass (scripts/start.sh start --vio), 'visual' (--no-gps), 'px4' with GPS
        DeclareLaunchArgument('odom', default_value='openvins'),
        DeclareLaunchArgument('camera_noise', default_value='true'),
        DeclareLaunchArgument('manhattan', default_value='true'),
        DeclareLaunchArgument('floor', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={'camera': 'true', 'camera_noise': LaunchConfiguration('camera_noise')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            launch_arguments={
                'bridge': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': 'false',
                'odom': LaunchConfiguration('odom'),
                'manhattan': LaunchConfiguration('manhattan'),
                'floor': LaunchConfiguration('floor'),
            }.items(),
        ),
        Node(
            package='daedalus_exploration',
            executable='auto_exploration',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='daedalus_trajectories',
            executable='offboard_control',
            # the exploration setpoints are in rtabmap's map frame: converted to PX4's frame
            parameters=[{'slam_correction': True}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_exploration',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
