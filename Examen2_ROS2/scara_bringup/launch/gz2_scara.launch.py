import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    pkg_scara_description = get_package_share_directory('scara_description')
    pkg_scara_bringup = get_package_share_directory('scara_bringup')

    urdf_path = os.path.join(pkg_scara_description, 'urdf', 'gz2_scara.xacro')
    gazebo_config_path = os.path.join(pkg_scara_bringup, 'config', 'gz_bridge.yaml')
    world_path = os.path.join(pkg_scara_bringup, 'worlds', 'world_config.world')

    rviz_config_path = os.path.join(pkg_scara_bringup, 'rviz', 'scara_config.rviz')

    use_sim_time = {'use_sim_time': True}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}, use_sim_time]
    )


    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}, use_sim_time],
        output='screen'
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path, '--render-engine', 'ogre'],
        output='screen'
    )

    spaw_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', 'robot_description',
        '-name', 'robot_scara',
        '-world', 'world_config' # DEBE coincidir con el <world name="..."> de tu archivo .world
    ],
    output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[use_sim_time] # Fundamental para evitar errores de TF
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spaw_entity,
        gz_ros_bridge_node,
        rviz_node
    ])
