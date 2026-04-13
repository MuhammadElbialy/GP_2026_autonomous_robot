from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 🔵 RPLidar Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser'
            }]
        ),

        # 🔴 Static TF: base_link → laser (FIXED ORIENTATION)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0', '0', '0',      # x y z
                '0', '3.14', '0',   # roll pitch yaw  ← 🔥 التعديل هنا
                'base_link',
                'laser'
            ],
            output='screen'
        )

    ])