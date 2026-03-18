import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('tm5_900')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tm5_900.urdf')

    # Read URDF text
    with open(urdf_path, 'r') as f:
        robot_desc_content = f.read()

    # 1. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': True}]
    )

    # 2. Launch Empty Gazebo World
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py'))
    )

    # 3. Spawn URDF in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tm5_900'],
        output='screen'
    )

    # 4. Spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # 5. Delay Spawners until Robot is spawned safely
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        delay_jsb, # JSB will trigger after spawn
        arm_spawner # Arm spawner can launch alongside
    ])