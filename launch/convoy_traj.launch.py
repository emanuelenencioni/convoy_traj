from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='zed_left_camera_optical_frame',
        description='Camera optical frame name'
    )
    
    tag0_frame_arg = DeclareLaunchArgument(
        'tag0_frame',
        default_value='tag36h11:0',
        description='First AprilTag frame name'
    )
    
    tag1_frame_arg = DeclareLaunchArgument(
        'tag1_frame',
        default_value='tag36h11:1',
        description='Second AprilTag frame name'
    )
    
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='front_vehicle',
        description='Output frame for the averaged pose'
    )
    
    timer_period_arg = DeclareLaunchArgument(
        'timer_period_ms',
        default_value='100',
        description='Timer period in milliseconds'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.8',
        description='Smoothing factor for exponential moving average (0.0 < alpha <= 1.0)'
    )
    
    # Covariance parameters
    position_covariance_arg = DeclareLaunchArgument(
        'position_covariance',
        default_value='0.1',
        description='Covariance for x, y, z position'
    )
    
    orientation_covariance_arg = DeclareLaunchArgument(
        'orientation_covariance',
        default_value='0.05',
        description='Covariance for roll, pitch, yaw orientation'
    )
    
    # Front axes offset parameters
    axes_offset_x_arg = DeclareLaunchArgument(
        'axes_offset_x',
        default_value='0.0',
        description='X offset from front_vehicle to front_axes (meters)'
    )
    
    axes_offset_y_arg = DeclareLaunchArgument(
        'axes_offset_y',
        default_value='0.0',
        description='Y offset from front_vehicle to front_axes (meters)'
    )
    
    axes_offset_z_arg = DeclareLaunchArgument(
        'axes_offset_z',
        default_value='0.0',
        description='Z offset from front_vehicle to front_axes (meters)'
    )
    
    # Create the convoy_traj_node
    convoy_traj_node = Node(
        package='convoy_traj',
        executable='convoy_traj_node',
        name='convoy_traj_node',
        output='screen',
        parameters=[{
            'camera_frame': LaunchConfiguration('camera_frame'),
            'tag0_frame': LaunchConfiguration('tag0_frame'),
            'tag1_frame': LaunchConfiguration('tag1_frame'),
            'output_frame': LaunchConfiguration('output_frame'),
            'timer_period_ms': LaunchConfiguration('timer_period_ms'),
            'alpha': LaunchConfiguration('alpha'),
            'position_covariance': LaunchConfiguration('position_covariance'),
            'orientation_covariance': LaunchConfiguration('orientation_covariance'),
            'axes_offset_x': LaunchConfiguration('axes_offset_x'),
            'axes_offset_y': LaunchConfiguration('axes_offset_y'),
            'axes_offset_z': LaunchConfiguration('axes_offset_z'),
        }]
    )

    return LaunchDescription([
        camera_frame_arg,
        tag0_frame_arg,
        tag1_frame_arg,
        output_frame_arg,
        timer_period_arg,
        alpha_arg,
        position_covariance_arg,
        orientation_covariance_arg,
        axes_offset_x_arg,
        axes_offset_y_arg,
        axes_offset_z_arg,
        convoy_traj_node
    ])
