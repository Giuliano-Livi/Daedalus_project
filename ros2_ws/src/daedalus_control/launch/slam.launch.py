import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    new_map = LaunchConfiguration('new_map').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    viz = LaunchConfiguration('rtabmap_viz').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    odom = LaunchConfiguration('odom').perform(context).lower()
    if odom not in ('px4', 'visual'):
        raise RuntimeError(f"odom:={odom} non valido, usare 'px4' o 'visual'")

    # ZED 2i simulata: colore + depth pixel-allineate (stessi intrinseci), entrambe a 15 Hz
    remaps = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        # rtabmap e rtabmap_viz stanno nel namespace /rtabmap, l'odometria pubblica su /odom
        ('odom', '/odom'),
    ]
    common = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        # colore e depth sono due sensori gz distinti: sync approssimato
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'wait_for_transform': 0.2,
        # drone: 6DoF completi
        'Reg/Force3DoF': 'false',
    }
    odom_params = {
        **common,
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'Vis/MinInliers': '15',
        # persa (es. muro senza texture davanti) -> reset al frame successivo: rtabmap apre una
        # nuova sessione di mappa e la riaggancia alla vecchia alla prima loop closure
        'Odom/ResetCountdown': '1',
    }
    slam_params = {
        **common,
        'map_frame_id': 'map',
        # nuovo nodo nel grafo ogni 10 cm o ~6 gradi
        'RGBD/LinearUpdate': '0.1',
        'RGBD/AngularUpdate': '0.1',
        'RGBD/ProximityBySpace': 'true',
        'Vis/MinInliers': '15',
        # mappa di occupazione dalla depth (/rtabmap/grid_map 2D, /rtabmap/cloud_map 3D)
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': '8.0',
        'Grid/CellSize': '0.1',
        # localization:=true -> usa la mappa salvata in ~/.ros/rtabmap.db senza estenderla
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }

    if odom == 'px4':
        # odometria dell'EKF di PX4 (IMU + GPS in SITL), non si perde mai: /odom + TF odom -> base_link
        odom_node = Node(
            package='daedalus_control',
            executable='px4_odometry',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    else:
        # odometria visuale RGB-D: /odom + TF odom -> base_link
        odom_node = Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            parameters=[odom_params],
            remappings=remaps,
            output='screen',
        )

    nodes = [
        odom_node,
        # graph SLAM: TF map -> odom, mappe (/rtabmap/cloud_map, /rtabmap/grid_map, ...),
        # database ~/.ros/rtabmap.db
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='rtabmap',
            parameters=[slam_params],
            remappings=remaps,
            # -d cancella il database all'avvio (solo in mapping)
            arguments=['-d'] if new_map and not localization else [],
            output='screen',
        ),
    ]
    if viz:
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            namespace='rtabmap',
            parameters=[common],
            remappings=remaps,
            output='screen',
        ))
    if rviz:
        # RViz2 con mappa 3D, occupancy grid, traiettoria SLAM e immagine colore
        rviz_cfg = os.path.join(get_package_share_directory('daedalus_control'), 'rviz', 'slam.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    sim_bringup = os.path.join(
        get_package_share_directory('daedalus_control'), 'launch', 'sim_bringup.launch.py')

    return LaunchDescription([
        # avvia bridge gz (clock, imu, camera) + TF della camera da sim_bringup:
        # false se sim_bringup.launch.py (o trajectory_following) e' gia' in esecuzione
        DeclareLaunchArgument('bridge', default_value='true'),
        # true: nuova mappa (cancella ~/.ros/rtabmap.db); false: continua la mappa salvata
        DeclareLaunchArgument('new_map', default_value='true'),
        # true: solo localizzazione sulla mappa salvata
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        # sorgente dell'odometria: 'px4' (EKF di PX4) o 'visual' (rgbd_odometry di RTAB-Map)
        DeclareLaunchArgument('odom', default_value='px4'),
        # RViz2 con la mappa in costruzione (rviz/slam.rviz)
        DeclareLaunchArgument('rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={'camera': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),
        OpaqueFunction(function=_setup),
    ])
