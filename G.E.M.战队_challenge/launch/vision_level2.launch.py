from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teamx_level2_cpp',
            executable='vision_node',
            name='vision_level2',
            output='screen',
            parameters=['config/params.yaml']
        )
    ])
