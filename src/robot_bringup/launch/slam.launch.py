from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ======================
    # 📦 Packages
    # ======================
    bringup_pkg = get_package_share_directory('robot_bringup')

    # ======================
    # 🔹 Odom + EKF + IMU
    # ======================
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'full_system.launch.py')
        )
    )

    # ======================
    # 🔹 Lidar
    # ======================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'lidar.launch.py')
        )
    )

    # ======================
    # 🔹 SLAM PARAM FILE (زي ما كان)
    # ======================
    slam_params_file = '/home/elbialy/GP_2026/src/robot_bringup/config/slam_params.yaml'

    # ======================
    # 🔥 SLAM TOOLBOX (زي ما كان)
    # ======================
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/elbialy/GP_2026/slam_config.rviz'],
        output='screen'
    )

    # ======================
    # 🚀 Launch
    # ======================
    return LaunchDescription([
        odom_launch,
        lidar_launch,
        slam,
        rviz_node
        
    ])