import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('lightning')

    # 声明参数
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg_dir, 'config', 'default.yaml'),
        description='配置文件路径'
    )

    # 获取配置文件路径
    config_file = LaunchConfiguration('config')

    # SLAM 节点
    slam_node = Node(
        package='lightning',
        executable='run_slam_online',
        name='lightning_slam',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        config_arg,
        slam_node,
    ])
