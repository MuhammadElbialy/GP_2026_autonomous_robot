import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # URDF/XACRO file
    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'final2.xml'
    )

    doc = xacro.process_file(xacro_file)

    robot_description = {
        "robot_description": doc.toxml()
    }

    # RViz config
    rviz_config_path = os.path.join(
        pkg_share,
        'rviz',
        'view_robot.rviz'
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # base_link -> base_footprint
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=[
            '0', '0', '0.31',
            '0', '0', '0', '1',
            'base_link',
            'base_footprint'
        ],
        output='screen'
    )

    # Joint State Publisher GUI
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rsp,
        base_tf,
        jsp,
        rviz
    ])
