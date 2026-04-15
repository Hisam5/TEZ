import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
# Gerekli kütüphaneler
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    pkg_name = 'ur16_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)
    ur16_pkg_share = get_package_share_directory('ur16') 

    # 1. URDF/XACRO dosyasını işleme
    robot_description_content = Command([
        'xacro ', os.path.join(ur16_pkg_share, 'urdf', 'ur16.urdf.xacro')
    ])

    # 2. Hata Düzeltme: robot_description'ı string olarak tanımla
    robot_description = {'robot_description': ParameterValue(robot_description_content, 
                                                            value_type=str)} 
                                                            
    # 3. Robot Durum Yayıncısı (URDF'i ROS'a yayınlar)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] 
    )
    
    # 4. Gazebo'yu başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': 'empty.world'}.items(), 
    )
    
    # 5. Robotu Gazebo'ya at (Spawn)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur16'],
                        output='screen')

    # 6. Kontrolcü Manager'ı başlat (Spawn'dan sonra çalışacak)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, os.path.join(pkg_share, 'config', 'ur16_controllers.yaml')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    
    # 7. Kontrolcü Yükleme Komutları (ExecuteProcess nesneleri)
    load_controllers = [
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/controller_manager/load_controller', 
                 'controller_manager_msgs/srv/LoadController', '{name: joint_state_broadcaster}'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/controller_manager/load_controller', 
                 'controller_manager_msgs/srv/LoadController', '{name: joint_trajectory_controller}'],
            output='screen'
        )
    ]

    # 8. Başlatma Sırası (Event Handler Zinciri)
    
    # 8a. Robot Spawn olduktan sonra Controller Manager'ı başlat
    start_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_manager], 
        )
    )

    # 8b. Controller Manager başladıktan sonra kontrolcüleri yükle
    load_controller_chain = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=load_controllers, 
        )
    )

    return LaunchDescription([
        # Gazebo'yu başlat
        gazebo,
        # Robotu at (Spawn)
        spawn_entity,
        # Robot Durum Yayıncısı
        node_robot_state_publisher,
        
        # KONTROL ZİNCİRİ
        start_controller_manager,
        load_controller_chain,
    ])
