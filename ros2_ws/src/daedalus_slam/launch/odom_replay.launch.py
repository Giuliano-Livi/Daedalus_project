import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Odometry only, on a recorded flight (scripts/odom_eval/run_replay.sh): the bag gives /clock, the ideal
# images (/sim/camera/...), camera_info, /imu/data (PX4), /zed/imu/data (ZED, bags recorded after it was
# added), /tf_static and the ground truth /gt/odom. Same right image, camera noise and estimator nodes as
# in flight, output on /vo/odom
def _setup(context):
    estimator = LaunchConfiguration('estimator').perform(context).lower()
    if estimator not in ('rgbd', 'openvins'):
        raise RuntimeError(f"estimator:={estimator} non valido, usare 'rgbd' o 'openvins'")
    share = get_package_share_directory('daedalus_slam')
    config = os.path.join(share, 'config', 'rtabmap.yaml')

    nodes = []
    # immagine destra della coppia stereo, come in volo (non e' nel bag: si rigenera qui)
    nodes.append(Node(
        package='daedalus_camera',
        executable='stereo_synth',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('color_in', '/sim/camera/color/image_raw'),
            ('depth_in', '/sim/camera/depth/image_raw'),
            ('camera_info_in', '/camera/color/camera_info'),
            ('right_out', '/sim/camera/right/image_raw'),
            ('right_info_out', '/camera/right/camera_info'),
        ],
        output='screen',
    ))
    # rumore ZED 2i come in volo (daedalus_trajectories/sim_bringup.launch.py), con seed fisso: ogni
    # stimatore vede lo stesso rumore, le differenze tra due replay vengono dallo stimatore
    for seed, (stream, kind) in enumerate((('color', 'color'), ('depth', 'depth'), ('right', 'color')), start=1):
        nodes.append(Node(
            package='daedalus_camera',
            executable='camera_noise',
            name=f'camera_noise_{stream}',
            parameters=[{'use_sim_time': True, 'stream': kind, 'seed': seed}],
            remappings=[
                (f'{kind}_in', f'/sim/camera/{stream}/image_raw'),
                (f'{kind}_out', f'/camera/{stream}/image_raw'),
            ],
            output='screen',
        ))
    if estimator == 'rgbd':
        # come slam.launch.py con odom:=visual
        nodes.append(Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            parameters=[config],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/vo/odom'),
                ('imu', '/imu/data'),
            ],
            output='screen',
        ))
    # prua rispetto alla struttura ortogonale del magazzino (scripts/odom_eval/eval_manhattan.py)
    nodes.append(Node(
        package='daedalus_slam',
        executable='manhattan_yaw',
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))
    if estimator == 'openvins':
        # come slam.launch.py con odom:=openvins
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'openvins.launch.py'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # stimatore da valutare: rgbd = rtabmap rgbd_odometry, openvins = VIO stereo con l'IMU della ZED
        DeclareLaunchArgument('estimator', default_value='rgbd'),
        OpaqueFunction(function=_setup),
    ])
