import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch mobile_manipulator_description display.launch.py                 # prototype layout


def nodes(context):
    share = get_package_share_directory('mobile_manipulator_description')
    layout = LaunchConfiguration('layout').perform(context)
    desc = open(os.path.join(share, 'urdf', f'mobile_manipulator_{layout}_ros.urdf')).read()
    rviz_cfg = os.path.join(share, 'rviz', 'display.rviz')   # RobotModel + TF, fixed frame base_link
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='prototype', description='prototype (the only layout since 2026-11-09)'),
        OpaqueFunction(function=nodes),
    ])
