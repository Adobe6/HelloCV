from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teamx_shooter_cpp',
            executable='player_pkg',
            name='shooter_node',
            output='screen',
            parameters=[{
                'fx': 800.0, 'fy': 800.0, 'cx': 320.0, 'cy': 240.0,
                'armor_w': 0.135, 'armor_h': 0.055,
                'bullet_v0': 25.0,
                'kp_yaw': 1.2, 'kp_pitch': 1.2,
                'eps_yaw': 0.01, 'eps_pitch': 0.01,
                'stable_frames': 3
            }]
        )
    ])
