import os

from ament_index_python.packages import get_package_share_directory

from ros_gz_bridge.actions import RosGzBridge

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
  
    world_name = 'test_world.sdf'
    world_true_name = world_name.rstrip('.sdf')
    
    av_navigation_ros_pkg_path = get_package_share_directory('av_navigation')
    av_simulation_ros_pkg_path = get_package_share_directory('av_simulation')
    ros_gz_sim_pkg_path        = get_package_share_directory('ros_gz_sim')
    
    nav_config_dir = os.path.join(av_navigation_ros_pkg_path, 'config')
    sim_config_dir = os.path.join(av_simulation_ros_pkg_path, 'config')

    world_file_path = os.path.join(av_simulation_ros_pkg_path, 'worlds', world_name)
    
    gz_bridge_config_path = os.path.join(sim_config_dir, 'config_gazebo_bridge.yaml')
    with open(gz_bridge_config_path, 'r') as f:
        config_content = f.read()
    new_config_content = config_content.replace("world_name", world_true_name)
    new_gz_bridge_config_path = os.path.join(sim_config_dir, 'config_gazebo_bridge_new.yaml')
    with open(new_gz_bridge_config_path, 'w') as f:
        f.write(new_config_content)

    gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-s ', world_file_path], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g '}.items()
    )
    
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=new_gz_bridge_config_path,
    )
    
    node_vision = Node(
        package='av_navigation',
        namespace='',
        executable='av_vision_node',
        name='av_vision_node',
        parameters=[os.path.join(nav_config_dir, 'config_vision_module.yaml')]
    )
    
    node_planner = Node(
        package='av_navigation',
        namespace='',
        executable='av_planner_node',
        name='av_planner_node',
        parameters=[os.path.join(nav_config_dir, 'config_planner_module.yaml')]
    )
    
    node_control = Node(
        package='av_navigation',
        namespace='',
        executable='av_control_node',
        name='av_control_node',
        parameters=[os.path.join(nav_config_dir, 'config_control_module.yaml')]
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(ros_gz_bridge)
    
    ld.add_action(node_vision)
    ld.add_action(node_planner)
    ld.add_action(node_control)

    return ld
