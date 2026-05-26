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
            parameters=[
                os.path.join(
                    get_package_share_directory('composite_cbf'),
                    'config', 'magpie.yaml'
                ),
            ],
            remappings=[
                ('~/obstacles', '/cbf_pc_selector/output_pc'),
                ('~/odom', '/msf_core/odometry_50hz'),
                # ('~/cmd_in', '/cmd_joy'),
                ('~/cmd_in', '/sdf_nmpc/cmd_acc'),
                ('~/safe_cmd_postarget', '/mavros/setpoint_raw/local'),
            ],
            output='screen'
        )
    ])
