# open3d_loc launch for G1 — ICP global localization against pre-built PCD
# Source: GitHub leokkzzhzzz/g1_3d_nav ros2 branch (canon)
# Local addition: DeclareLaunchArgument('map_file') so the PCD path is overridable
# from `ros2 launch ... map_file:=/path/to.pcd` without editing this file.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # G1: PCD map path is parameterised. Default points at the FAST-LIO mapping
    # output baked into /g1_3d_nav_ros2/maps/scans.pcd. Override with map_file:=...
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/g1_3d_nav_ros2/maps/scans.pcd',
        description='Path to PCD map file for ICP localization'
    )
    map_file = LaunchConfiguration('map_file')

    config_file = PathJoinSubstitution([
        open3d_loc_share, 'config', 'loc_param_g1.yaml'
    ])

    # Static TF: odom -> camera_init (FAST-LIO world frame)
    static_tf_camera_init2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    # Static TF: base_link -> imu_link (G1: matches ROS1 direction)
    static_tf_imulink2baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    # Static TF: motion_link -> base_link (G1: matches ROS1 direction)
    static_tf_base_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'motion_link', 'base_link']
    )

    # Global localization: ICP registration of current scan against pre-built PCD
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        # Remap: the C++ node has a hardcoded /scan PointCloud2 publisher that
        # collides with pointcloud_to_laserscan's /scan LaserScan. FastDDS
        # tolerates same-name-different-type; rmw_zenoh_cpp does not (topic
        # echo /scan reports "more than one type"). Move to /scan_loc so the
        # standard /scan namespace is left to pcl2scan.
        remappings=[('/scan', '/scan_loc')],
        parameters=[
            config_file,
            {
                'path_map': map_file,
                # G1: tuned ICP and localization parameters
                'pcd_queue_maxsize': 10,
                # G1: voxelsize_coarse must be >= 0.15 for 8M-point PCD (0.01 causes empty voxels)
                'voxelsize_coarse': 0.15,
                # G1: voxelsize_fine=0.2 -> MultiScaleIcp scales {0.2,0.4,0.6}
                # matching ROS1 coarsest voxel 0.6 for robust sparse-point matching
                'voxelsize_fine': 0.2,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        static_tf_camera_init2odom,
        static_tf_imulink2baselink,
        static_tf_base_center,
        global_localization_node,
    ])
