import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # 1. PAKET VE DOSYA YOLLARI
    pkg_share = get_package_share_directory('ur16')
    
    # ÖNEMLİ: Artık .urdf değil .xacro dosyasını okuyoruz!
    default_model_path = os.path.join(pkg_share, 'urdf', 'ur16.urdf.xacro')
    
    # RViz ayar dosyası (Eğer varsa kullanır, yoksa varsayılanı açar)
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')

    # 2. LAUNCH ARGÜMANLARI
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Robotun URDF/XACRO dosyasının tam yolu'
    )

    # 3. ROBOT STATE PUBLISHER (Robotun Vücudu)
    # XACRO'yu işleyip ROS'a "robot_description" olarak sunar.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 4. JOINT STATE PUBLISHER GUI (Slider Penceresi)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 5. RVIZ2 (Görselleştirme)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Eğer 'config/display.rviz' dosyan varsa alttaki satırın başındaki # işaretini kaldır
        # arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
