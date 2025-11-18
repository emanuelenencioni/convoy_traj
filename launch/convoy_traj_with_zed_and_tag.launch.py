from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file paths
    apriltag_config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )
    
    convoy_traj_config = os.path.join(
        get_package_share_directory('convoy_traj'),
        'config',
        'convoy_traj_params.yaml'
    )
    
    # Get ZED launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
        }.items()
    )
    
    # Declare launch arguments for convoy_traj
    axes_offset_x_arg = DeclareLaunchArgument(
        'axes_offset_x',
        default_value='0.0',
        description='X offset from front_vehicle to front_axes'
    )
    
    axes_offset_y_arg = DeclareLaunchArgument(
        'axes_offset_y',
        default_value='0.0',
        description='Y offset from front_vehicle to front_axes'
    )
    
    axes_offset_z_arg = DeclareLaunchArgument(
        'axes_offset_z',
        default_value='0.0',
        description='Z offset from front_vehicle to front_axes'
    )
    
    # Load both AprilTag and Convoy nodes into ZED's container
    load_composable_nodes = LoadComposableNodes(
        target_container='/zed/zed_container',
        composable_node_descriptions=[
            # AprilTag Node
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag_node',
                parameters=[apriltag_config, {'size': 0.12}],
                remappings=[
                    ('image', '/zed/zed_node/left/color/raw/image'),
                    ('camera_info', '/zed/zed_node/left/color/raw/image/camera_info'),
                    ('detections', '/apriltag_node/detections'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Convoy Trajectory Node
            ComposableNode(
                package='convoy_traj',
                plugin='convoy_traj::ConvoyTrajNode',
                name='convoy_traj_node',
                parameters=[
                    convoy_traj_config,
                    {
                        'axes_offset_x': LaunchConfiguration('axes_offset_x'),
                        'axes_offset_y': LaunchConfiguration('axes_offset_y'),
                        'axes_offset_z': LaunchConfiguration('axes_offset_z'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    
    # Delay loading to ensure ZED container is ready
    delayed_load = TimerAction(
        period=3.0,  # Wait 3 seconds for ZED to fully start
        actions=[load_composable_nodes]
    )

    return LaunchDescription([
        axes_offset_x_arg,
        axes_offset_y_arg,
        axes_offset_z_arg,
        zed_wrapper_launch,
        delayed_load,
    ])
