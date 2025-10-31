from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('convoy_traj'),
        'config',
        'convoy_traj_params.yaml'
    )
    
    # Create the convoy_traj_node with parameters from YAML file
    convoy_traj_node = Node(
        package='convoy_traj',
        executable='convoy_traj_node',
        name='convoy_traj_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        convoy_traj_node
    ])
