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
        get_package_share_directory('daedalus_control'), 'launch', 'sim_bringup.launch.py')

    return LaunchDescription([
        # starts the gz bridge (clock, imu, camera) from sim_bringup:
        # set it to false if sim_bringup.launch.py is already running, to avoid a duplicate bridge
        DeclareLaunchArgument('bridge', default_value='true'),
        # save the camera images while following the trajectory (only when requested: save_images:=true)
        DeclareLaunchArgument('save_images', default_value='false'),
        DeclareLaunchArgument('save_depth', default_value='false'),
        DeclareLaunchArgument('image_rate', default_value='2.0'),
        # /ws/images is mounted on the host as <project>/images (docker-compose.yml)
        DeclareLaunchArgument('image_dir', default_value='/ws/images'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={'camera': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('bridge')),
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
        Node(
            package='daedalus_control',
            executable='image_saver',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('image_dir'),
                'rate': ParameterValue(LaunchConfiguration('image_rate'), value_type=float),
                'save_depth': ParameterValue(LaunchConfiguration('save_depth'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('save_images')),
        ),
    ])
