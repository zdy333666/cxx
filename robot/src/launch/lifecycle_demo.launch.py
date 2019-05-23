from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='lifecycle', node_executable='lifecycle_talker',
                      node_name='lc_talker', output='screen'),
        Node(package='lifecycle', node_executable='lifecycle_listener', output='screen'),
        Node(package='lifecycle', node_executable='lifecycle_service_client', output='screen')
    ])
