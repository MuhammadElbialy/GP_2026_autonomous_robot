from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('encoder_odometry')

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'odom_config.rviz')
    ekf_config_path  = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 🔥 مهم جدًا للتأكد إن الـ path صح
    print("EKF CONFIG PATH:", ekf_config_path)

    return LaunchDescription([

        # =========================
        # 🔹 Encoder + IMU Node
        # =========================
        Node(
            package='encoder_odometry',
            executable='encoder_odom_node',
            name='encoder_odometry_node',
            parameters=[{
                'port': '/dev/ttyACM0'
            }],
            output='screen'
        ),

        # =========================
        # 🔹 EKF (robot_localization)
        # =========================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            emulate_tty=True   # 🔥 مهم عشان يظهر logs
        ),

        # =========================
        # 🔹 IMU TF
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=[
                '0.30', '0.10', '0.47',
                '0', '0', '0',
                'base_link',
                'imu_link'
            ],
            output='screen'
        ),

        # =========================
        # 🔹 RViz
        # =========================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])