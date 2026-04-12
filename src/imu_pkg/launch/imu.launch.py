from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # IMU Node
        Node(
            package='imu_pkg',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        # IMU TF (base_link → imu_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.30', '0.10', '0.47', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
    ])
