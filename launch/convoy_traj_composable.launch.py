from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file paths
    apriltag_config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )
    
    zed_common_config = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common_stereo.yaml'
    )
    
    zed_camera_config = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zed.yaml'  # Change to your camera model: zed.yaml, zed2.yaml, zedm.yaml, etc.
    )
    # Declare launch arguments
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

    # Create composable node container
    container = ComposableNodeContainer(
        name='convoy_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            # ZED Camera Node
            ComposableNode(
                package='zed_wrapper',
                plugin='stereolabs::ZedCamera',
                name='zed_node',
                namespace='zed',
                parameters=[
                    zed_common_config,
                    zed_camera_config,
                    {
                        'general.camera_name': 'zed2i',
                        'general.camera_model': 'zed2i',
                    }
                ],
            ),
            
            # AprilTag Detection Node
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag_node',
                remappings=[
                    ('image_rect', '/zed/zed_node/left/image_rect_color'),
                    ('camera_info', '/zed/zed_node/left/camera_info'),
                ],
                parameters=[
                    apriltag_config,
                    {
                        'size': 0.12,
                        'image_transport': 'raw',
                    }
                ],
            ),
            
            # Convoy Trajectory Node
            ComposableNode(
                package='convoy_traj',
                plugin='convoy_traj::ConvoyTrajNode',
                name='convoy_traj_node',
                parameters=[{
                    'axes_offset_x': LaunchConfiguration('axes_offset_x'),
                    'axes_offset_y': LaunchConfiguration('axes_offset_y'),
                    'axes_offset_z': LaunchConfiguration('axes_offset_z'),
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        axes_offset_x_arg,
        axes_offset_y_arg,
        axes_offset_z_arg,
        container,
    ])
