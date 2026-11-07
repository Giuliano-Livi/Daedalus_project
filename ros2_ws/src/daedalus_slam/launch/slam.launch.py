import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    bridge = LaunchConfiguration('bridge').perform(context).lower() == 'true'
    new_map = LaunchConfiguration('new_map').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    viz = LaunchConfiguration('rtabmap_viz').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    odom = LaunchConfiguration('odom').perform(context).lower()
    if odom not in ('px4', 'visual'):
        raise RuntimeError(f"odom:={odom} non valido, usare 'px4' o 'visual'")

    share = get_package_share_directory('daedalus_slam')
    # parametri RTAB-Map (frame, sync, aggiornamento del grafo, griglia): config/rtabmap.yaml
    config = os.path.join(share, 'config', 'rtabmap.yaml')

    # ZED 2i simulata: colore + depth pixel-allineate (stessi intrinseci), entrambe a 15 Hz
    remaps = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        # rtabmap e rtabmap_viz stanno nel namespace /rtabmap, l'odometria pubblica su /odom
        ('odom', '/odom'),
    ]

    nodes = []
    if bridge:
        # bridge gz (clock, imu, camera) + TF della camera da daedalus_trajectories. Cercato solo qui,
        # a runtime: daedalus_trajectories include gia' questo launch (niente dipendenza circolare)
        sim_bringup = os.path.join(
            get_package_share_directory('daedalus_trajectories'), 'launch', 'sim_bringup.launch.py')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_bringup),
            launch_arguments={
                'camera': 'true',
                'camera_noise': LaunchConfiguration('camera_noise').perform(context),
            }.items(),
        ))

    # odometria per rtabmap: sempre quella dell'EKF di PX4 (/odom + TF odom -> base_link).
    # odom:=px4 -> IMU + GPS in SITL; odom:=visual -> IMU + odometria visuale (+ baro, bussola):
    # quando la visione si perde per poco PX4 prosegue con l'IMU, quindi rtabmap riceve una posa
    # continua (con l'odometria visuale diretta ogni perdita faceva un reset e una nuova sessione
    # di mappa staccata dalla precedente)
    nodes.append(Node(
        package='daedalus_slam',
        executable='px4_odometry',
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))
    if odom == 'visual':
        # odometria visuale RGB-D su /vo/odom, senza TF (la TF odom -> base_link e' di
        # px4_odometry). L'IMU del drone (250 Hz, frame base_link) allinea l'odometria alla
        # gravita' e vincola rollio/beccheggio
        nodes.append(Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            parameters=[config],    # publish_tf false, odom_frame_id vo_odom: rtabmap.yaml
            remappings=[r for r in remaps if r[0] != 'odom'] + [('odom', '/vo/odom'), ('imu', '/imu/data')],
            output='screen',
        ))
        # e la manda all'EKF di PX4 come external vision: unica sorgente di posizione senza GPS
        # (scripts/start.sh --no-gps); con il GPS attivo PX4 la ignora (EKF2_EV_CTRL = 0)
        nodes.append(Node(
            package='daedalus_slam',
            executable='vision_to_px4',
            parameters=[{'use_sim_time': True}],
            remappings=[('/odom', '/vo/odom')],
            output='screen',
        ))

    # graph SLAM: TF map -> odom, mappe (/rtabmap/cloud_map, /rtabmap/grid_prob_map, ...),
    # database ~/.ros/rtabmap.db
    # localization:=true -> usa la mappa salvata in ~/.ros/rtabmap.db senza estenderla
    memory = {
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }
    # nuvola 3D dalla depth (rumorosa), salvata in ogni nodo del grafo: le loop closure trovate
    # con le immagini vengono rifinite con ICP sulla struttura 3D (Reg/Strategy 2, rtabmap.yaml)
    nodes.append(Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': True,
            'decimation': 4,       # VGA 672x376 -> 168x94 punti
            'voxel_size': 0.05,
            'max_depth': 5.0,      # come Vis/MaxDepth: oltre la depth e' troppo rumorosa
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', 'scan_cloud'),
        ],
        output='screen',
    ))
    nodes.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        namespace='rtabmap',
        parameters=[config, memory],
        remappings=remaps,
        # -d cancella il database all'avvio (solo in mapping)
        arguments=['-d'] if new_map and not localization else [],
        output='screen',
    ))

    if viz:
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            namespace='rtabmap',
            parameters=[config],
            remappings=remaps,
            output='screen',
        ))
    if rviz:
        # RViz2 con mappa 3D, occupancy grid, traiettoria SLAM e immagine colore
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            arguments=['-d', os.path.join(share, 'rviz', 'slam.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # avvia bridge gz (clock, imu, camera) + TF della camera da daedalus_trajectories/sim_bringup:
        # false se sim_bringup.launch.py (o trajectory_following) e' gia' in esecuzione
        DeclareLaunchArgument('bridge', default_value='true'),
        # rumore realistico ZED 2i sulle immagini (solo con bridge:=true, altrimenti lo decide
        # chi ha avviato sim_bringup)
        DeclareLaunchArgument('camera_noise', default_value='false'),
        # true: nuova mappa (cancella ~/.ros/rtabmap.db); false: continua la mappa salvata
        DeclareLaunchArgument('new_map', default_value='true'),
        # true: solo localizzazione sulla mappa salvata
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        # sorgente dell'odometria: 'px4' (EKF di PX4) o 'visual' (rgbd_odometry di RTAB-Map)
        DeclareLaunchArgument('odom', default_value='px4'),
        # RViz2 con la mappa in costruzione (rviz/slam.rviz)
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
