from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 🔵 RPLidar Node (FIXED)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',

                # 🔥 الحل الأساسي للمشكلة
                'angle_compensate': True,

                # 🔧 إضافات stability
                'scan_mode': 'Sensitivity'
            }]
        ),

        # 🔴 Static TF: base_link → laser (KEEP AS IS)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0', '0', '0',
                '0', '3.14', '0',
                'base_link',
                'laser_frame'
            ],
            output='screen'
        )

    ])