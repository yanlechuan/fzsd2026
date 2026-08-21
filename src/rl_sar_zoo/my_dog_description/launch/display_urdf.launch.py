import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包的共享目录路径
    pkg_share_path = get_package_share_directory('my_dog_description')
    
    # 修正URDF文件路径 - 文件直接位于包共享目录下，不在urdf子目录
    urdf_path = os.path.join(pkg_share_path, 'my_dog_description.urdf')
    
    # 从文件读取URDF内容
    try:
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        raise Exception(f"URDF文件未找到: {urdf_path}")
    
    # 定义需要启动的节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])