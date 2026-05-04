import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

PACKAGE_NAME = 'tm5_900'

def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_withgripper.urdf')

    with open(urdf_path, 'r') as file:
        robot_description_xml = file.read()
        
    spawn_URDF = URDFSpawner(
        name='tm5_900',
        urdf_path=urdf_path,
        translation='0 0 0.64',
        rotation='0 0 1 0',
    )

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'] + controller_manager_timeout
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['arm_controller', '-c', '/controller_manager'] + controller_manager_timeout,
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['gripper_controller', '-c', '/controller_manager'] + controller_manager_timeout,
    )

    # ✅ righthand_joint_controller spawner added for mimic behavior
    righthand_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['righthand_joint_controller', '-c', '/controller_manager'] + controller_manager_timeout,
    )

    # ✅ mimic node to mirror lefthand_joint → righthand_joint
    mimic_node = Node(
        package='tm5_900',
        executable='mimic_joint_node',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
       parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True    
        }],
    )
    
    spawners = [jsb_spawner, arm_spawner, gripper_spawner, righthand_spawner, mimic_node]

    return [
        spawn_URDF,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF,
                on_stdout=lambda event: get_webots_driver_node(
                    event,[robot_state_publisher] + spawners
                ),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True,
        stream='mjpeg'
        
    )

    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_withgripper.urdf')
    controller_config = os.path.join(package_dir, 'config', 'ros2_controllersWithGripper.yaml')

    with open(urdf_path, 'r') as file:
        robot_description_xml = file.read()

    driver = WebotsController(
        robot_name='tm5_900',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': True},
            {'set_robot_state_publisher': False},
            controller_config
        ],
        respawn=True
    )

    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(  
            'world',
            default_value='industrial_world.wbt',
            description='Choose one of the world files from `/tm5_900/worlds` directory'
        ),
        webots,
        webots._supervisor,
        driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
        reset_handler
    ] + get_ros2_nodes())