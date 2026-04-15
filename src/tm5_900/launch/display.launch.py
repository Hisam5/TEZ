import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    
    package_dir = get_package_share_directory('tm5_900')
    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_900.urdf.xacro')
    world_path = os.path.join(package_dir, 'worlds', 'empty_world.wbt')

    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    webots = WebotsLauncher(world=world_path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Robotumuz zaten dünyada var, biz sadece sinir sistemini (kontrolcüyü) bağlıyoruz
    webots_driver = WebotsController(
        robot_name='tm5_900',
        parameters=[{'robot_description': urdf_path}]
    )

    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        webots,
        robot_state_publisher,
        webots_driver,
        shutdown_handler
    ])