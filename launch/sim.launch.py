from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='composite_cbf',
            executable='composite_cbf_node',
            name='composite_cbf',
            parameters=[{
                'config': os.path.join(
                    get_package_share_directory('composite_cbf'),
                    'config', 'sim.yaml'
                ),
            }],
            remappings=[
                ('~/obstacles', '/cbf_pc_selector/output_pc'),
                ('~/odom', '/rmf/odom'),
                ('~/safe_cmd_twist', '/rmf/cmd/acc'),
            ],
            output='screen'
        )
    ])
