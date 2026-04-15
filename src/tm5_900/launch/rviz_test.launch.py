import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('tm5_900')
    urdf_path = os.path.join(package_dir, 'urdf', 'tm5_withgripper.urdf')

    # URDF'i okuyoruz
    with open(urdf_path, 'r') as f:
        robot_desc_content = f.read()

    # 1. İskelet Yayıncısı (Robot State Publisher)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content}]
    )

    # 2. Kaydırma Çubuklu Arayüz (Joint State Publisher GUI)
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 3. RViz2'nin kendisi
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        rsp,
        jsp_gui,
        rviz
    ])