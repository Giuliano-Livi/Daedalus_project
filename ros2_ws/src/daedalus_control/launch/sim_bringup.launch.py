import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    takeoff = LaunchConfiguration('takeoff').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    pointcloud = LaunchConfiguration('pointcloud').perform(context).lower() == 'true'
    camera = camera or pointcloud

    nodes = []
    if sim:
        link = f'/world/{world}/model/{model}/link'
        # OAK-D Lite: colour + depth camera pair, pixel-aligned (gazebo/models/x500_depth)
        color = f'{link}/camera_link/sensor/color_camera'
        depth = f'{link}/camera_link/sensor/depth_camera'
        imu = f'{link}/base_link/sensor/imu_sensor/imu'

        # (topic Gazebo, topic ROS stabile, tipo ROS, tipo Gazebo)
        bridges = [
            ('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
            (imu, '/imu/data', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
        ]
        if camera:
            bridges.extend([
                (f'{color}/image', '/camera/color/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                (f'{color}/camera_info', '/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
                (f'{depth}/depth_image', '/camera/depth/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                (f'{depth}/depth_image/points', '/camera/depth/points',
                 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
                (f'{depth}/camera_info', '/camera/depth/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
            ])
        args = [f'{gz}@{ros_t}[{gz_t}' for gz, _, ros_t, gz_t in bridges]
        remaps = [(gz, ros) for gz, ros, _, _ in bridges if gz != ros]
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=args,
            remappings=remaps,
            output='screen',
        ))

    if sim and pointcloud:
        # RViz2 con point cloud, immagine colore e depth (finestra sul desktop host, vedi docker-compose dev)
        rviz_cfg = os.path.join(get_package_share_directory('daedalus_control'), 'rviz', 'depth_camera.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ))

    if takeoff:
        nodes.append(Node(
            package='daedalus_control',
            executable='takeoff_1m',
            name='takeoff_1m',
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse_px4'),
        DeclareLaunchArgument('model', default_value='x500_depth_0'),
        DeclareLaunchArgument('takeoff', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        # point cloud della depth camera in RViz2 (attiva anche i bridge della camera)
        DeclareLaunchArgument('pointcloud', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
