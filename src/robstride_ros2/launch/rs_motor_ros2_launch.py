from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rs_motor_ros2')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    node = Node(
        package='rs_motor_ros2',
        executable='rs_motor_ros2_node',
        name='rs_motor_ros2_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([node])
