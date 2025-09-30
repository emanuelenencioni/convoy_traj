from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_convoy_traj = get_package_share_directory('convoy_traj')

    # Start the convoy_traj_node
    convoy_traj_node = Node(
        package='convoy_traj',
        executable='convoy_traj_node',
        name='convoy_traj_node',
        output='screen'
    )

    # Start the navsat_transform_node
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{'use_odometry_yaw': False,
                     'magnetic_declination_radians': 0.0, # Adjust to your location
                     'yaw_offset': 1.5707963, # Adjust as needed
                     'zero_altitude': True,
                     'publish_filtered_gps': True,
                     'frequency': 30.0
                     }],
        remappings=[('/gps/fix', '/gnss/fix')]
    )

    # Start the UKF node
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        parameters=[os.path.join(pkg_convoy_traj, 'config', 'ukf.yaml')]
    )

    return LaunchDescription([
        convoy_traj_node,
        navsat_transform_node,
        ukf_node
    ])
