from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('encoder_odometry')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'odom_config.rviz')

    return LaunchDescription([

        # 🔹 ODOM NODE
        Node(
            package='encoder_odometry',
            executable='encoder_odom_node',
            name='encoder_odometry_node',
            parameters=[{'port': '/dev/ttyACM0'}],
            output='screen'
        ),

        # # 🔹 IMU NODE
        # Node(
        #     package='imu_pkg',
        #     executable='imu_node',
        #     name='imu_node',
        #     output='screen'
        # ),

        # 🔹 IMU TF (صح هندسيًا)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=[
                '0.30',  # X (عدّل حسب القياس)
                '0.10',  # Y
                '0.47',  # Z (انت قلت 47cm)
                '0', '0', '0',  # rotation (هنعدله بعد شوية)
                'base_link',
                'imu_link'
            ],
            output='screen'
        ),

        # 🔹 RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])