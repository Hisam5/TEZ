import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='tm5_900').find('tm5_900')
    
    # Hedefimiz artık xacro dosyası
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'tm5_900.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz') 

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # SİHİRLİ KISIM BURASI: Dosyayı metin olarak okumak yerine, 'xacro' komutunu çalıştırıp çıktısını alıyoruz
    robot_desc = Command(['xacro ', xacro_file_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Eğer config dosyan yoksa hata vermemesi için küçük bir kontrol ekledik
    if os.path.exists(rviz_config_path):
        rviz_args = ['-d', rviz_config_path]
    else:
        rviz_args = []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])