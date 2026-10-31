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
        get_package_share_directory('daedalus_control'), 'launch', 'sim_bringup.launch.py')
    slam = os.path.join(
        get_package_share_directory('daedalus_control'), 'launch', 'slam.launch.py')

    return LaunchDescription([
        # starts the gz bridge (clock, imu, camera) from sim_bringup:
        # set it to false if sim_bringup.launch.py is already running, to avoid a duplicate bridge
        DeclareLaunchArgument('bridge', default_value='true'),
        # RTAB-Map SLAM in parallelo al volo (slam:=true); il bridge resta quello incluso qui sotto
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('odom', default_value='px4'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={'camera': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            launch_arguments={
                'bridge': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'odom': LaunchConfiguration('odom'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
        Node(
            package='daedalus_control',
            executable='trajectory_generator',
            output='screen'
        ),
        Node(
            package='daedalus_control',
            executable='offboard_control',
            output='screen'
        ),
    ])
