import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    ur16_pkg_share = get_package_share_directory('ur16') 

    # 1. URDF/XACRO dosyasını işleme
    robot_description_content = Command([
        'xacro ', os.path.join(ur16_pkg_share, 'urdf', 'ur16.urdf.xacro')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, 
                                                            value_type=str)} 
                                                            
    # 2. Gazebo'yu başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': 'empty.world'}.items(), 
    )
    
    # 3. Robot Durum Yayıncısı
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] 
    )
    
    # 4. Robotu Gazebo'ya at (Spawn)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur16'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
