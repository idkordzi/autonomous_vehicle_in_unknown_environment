import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('av_navigation_ros')
    config_dir = os.path.join(pkg_path, 'config')

    node_vision = Node(
        package='av_navigation_ros',
        namespace='',
        executable='av_vision_node',
        name='av_vision_node',
        parameters=[os.path.join(config_dir, 'config_vision_module.yaml')]
    )

    node_planner = Node(
        package='av_navigation_ros',
        namespace='',
        executable='av_planner_node',
        name='av_planner_node',
        parameters=[os.path.join(config_dir, 'config_planner_module.yaml')]
    )

    node_ekf = Node(
        package='robot_localization',
        namespace='',
        executable='ekf_node',
        name='av_ekf_node',
        parameters=[os.path.join(config_dir, 'config_ekf.yaml')]
    )

    node_estimation = Node(
        package='av_navigation_ros',
        namespace='',
        executable='av_estimation_node',
        name='av_estimation_node',
        parameters=[os.path.join(config_dir, 'config_estimation_module.yaml')]
    )

    node_control = Node(
        package='av_navigation_ros',
        namespace='',
        executable='av_control_node',
        name='av_control_node',
        parameters=[os.path.join(config_dir, 'config_control_module.yaml')]
    )

    mock_publisher_node = Node(
        package='av_navigation_ros',
        namespace='',
        executable='mock_publisher_node',
        name='mock_publisher_node',
        parameters=[os.path.join(config_dir, 'config_mock_publisher.yaml')]
    )

    ld = LaunchDescription()

    ld.add_action(node_vision)
    ld.add_action(node_planner)
    ld.add_action(node_ekf)
    ld.add_action(node_estimation)
    ld.add_action(node_control)
    # ld.add_action(mock_publisher_node) # use to provide simple input data

    return ld
