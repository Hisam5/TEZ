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

# Simülasyon sıfırlandığında yeniden başlatılacak olan ROS 2 düğümleri
def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_900.urdf')

    # URDF Robotunu Simülasyona Enjekte Et (Spawner)
    spawn_URDF = URDFSpawner(
        name='tm5_900',
        urdf_path=urdf_path,
        translation='0 0 0',
        rotation='0 0 1 0',
    )

    # ROS Control Kas Başlatıcıları (Spawners)
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'] + controller_manager_timeout,
    )
    
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['arm_controller', '-c', 'controller_manager'] + controller_manager_timeout,
    )
    
    spawners = [jsb_spawner, arm_spawner]

    # İskelet (TF) Yayıncısı: WebotsController bunu otomatik olarak dolduracak
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    return [
        # 1. Önce robot bedenini oluştur
        spawn_URDF,

        # 2. Beden yaratılmayı bitirdiğinde, Sürücü düğümünü (Beyni) ve kasları güvenle başlat
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF,
                on_stdout=lambda event: get_webots_driver_node(
                    event, [
                        robot_state_publisher
                    ] + spawners
                ),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    # Webots Simülatörünü Başlat (Supervisor aktif)
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Sürücü (Driver Node)
    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_900.urdf')
    controller_config = os.path.join(package_dir, 'config', 'ros2_controllers.yaml')
    
    driver = WebotsController(
        robot_name='tm5_900',
        parameters=[
            {'robot_description': urdf_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            controller_config
        ],
        respawn=True
    )

    # Simülasyon Resetlendiğinde Düğümleri Yeniden Başlatma İşleyicisi
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty_world.wbt',
            description='Choose one of the world files from `/tm5_900/worlds` directory'
        ),
        webots,
        webots._supervisor,
        
        driver,

        # Webots kapatıldığında terminaldeki tüm süreçleri temizle
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

        # Reset olayını dinle
        reset_handler
    ] + get_ros2_nodes())