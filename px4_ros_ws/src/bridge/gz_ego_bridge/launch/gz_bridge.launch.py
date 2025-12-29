from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('gz_ego_bridge')
    cfg = os.path.join(share, 'config', 'bridge_config.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={cfg}']
    )

    # 让所有节点默认用仿真时间（也可以只对某些节点设置）
    use_sim_time = Node(
        package='rclcpp_components',
        executable='component_container',
        name='use_sim_time_dummy',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([bridge, use_sim_time])
