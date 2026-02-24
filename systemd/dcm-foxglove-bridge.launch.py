import socket
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    namespace = socket.gethostname()  # e.g. 'dcm0001'
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            namespace=namespace,
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'capabilities': ['clientPublish', 'services', 'connectionGraph'],
                'min_qos_depth': 1,
                'max_qos_depth': 10,
                'ignore_unresponsive_param_nodes': True,
            }]
        )
    ])
