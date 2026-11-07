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
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    pointcloud = LaunchConfiguration('pointcloud').perform(context).lower() == 'true'
    camera = camera or pointcloud
    noise = LaunchConfiguration('camera_noise').perform(context).lower() == 'true'
    # con il rumore le immagini ideali di gz vanno su /sim/camera/... e camera_noise pubblica quelle
    # rumorose su /camera/...: chi le usa (RTAB-Map, RViz) non cambia. camera_info resta su /camera
    ideal = '/sim/camera' if noise else '/camera'

    nodes = []
    if sim:
        link = f'/world/{world}/model/{model}/link'
        # ZED 2i (SDK output): colour + depth camera pair, pixel-aligned (gazebo/models/x500_depth)
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
                (f'{color}/image', f'{ideal}/color/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                (f'{color}/camera_info', '/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
                (f'{depth}/depth_image', f'{ideal}/depth/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                (f'{depth}/depth_image/points', f'{ideal}/depth/points',
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

    if sim and camera:
        # TF statiche della ZED 2i (gazebo/models/x500_depth/model.sdf): camera_link = lente sinistra
        # (montaggio .12 0 .256 + sensore .015 .06 0); camera_optical_frame = convenzione ottica
        # (z avanti, x destra, y giu'), il frame_id di immagini e camera_info
        static_tfs = [
            ['--x', '0.135', '--y', '0.06', '--z', '0.256',
             '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
            ['--roll', '-1.5707963', '--yaw', '-1.5707963',
             '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'],
        ]
        for i, tf_args in enumerate(static_tfs):
            nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'camera_tf_{i}',
                arguments=tf_args,
                parameters=[{'use_sim_time': True}],
                output='screen',
            ))

    if sim and camera and noise:
        # rumore ZED 2i (VGA) su colore e depth, parametri in daedalus_camera/src/camera_noise.cpp.
        # Un processo per immagine: ognuno ha il suo core
        for stream in ('color', 'depth'):
            nodes.append(Node(
                package='daedalus_camera',
                executable='camera_noise',
                name=f'camera_noise_{stream}',
                parameters=[{'use_sim_time': True, 'stream': stream}],
                remappings=[
                    (f'{stream}_in', f'/sim/camera/{stream}/image_raw'),
                    (f'{stream}_out', f'/camera/{stream}/image_raw'),
                ],
                output='screen',
            ))
        if pointcloud:
            # la point cloud di gz e' quella ideale: la ricostruisce dalla depth rumorosa
            nodes.append(Node(
                package='rtabmap_util',
                executable='point_cloud_xyz',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('depth/image', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('cloud', '/camera/depth/points'),
                ],
                output='screen',
            ))

    if sim and pointcloud:
        # RViz2 con point cloud, immagine colore e depth (finestra sul desktop host, vedi docker-compose dev)
        rviz_cfg = os.path.join(get_package_share_directory('daedalus_trajectories'), 'rviz', 'depth_camera.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse_px4'),
        DeclareLaunchArgument('model', default_value='x500_depth_0'),
        DeclareLaunchArgument('camera', default_value='false'),
        # point cloud della depth camera in RViz2 (attiva anche i bridge della camera)
        DeclareLaunchArgument('pointcloud', default_value='false'),
        # rumore realistico ZED 2i sulle immagini (daedalus_camera/camera_noise)
        DeclareLaunchArgument('camera_noise', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
