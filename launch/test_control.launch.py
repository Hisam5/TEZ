import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('tm5_900')
    
    # DİKKAT 1: .xacro DEĞİL, terminalde fırınlayacağımız saf .urdf dosyasını veriyoruz!
    urdf_path = os.path.join(package_dir, 'urdf', 'test_mock.urdf')
    controller_config = os.path.join(package_dir, 'config', 'ros2_controllers.yaml')

    # DİKKAT 2: Command() ile Xacro çalıştırmıyoruz, direkt metni okuyoruz.
    with open(urdf_path, 'r') as f:
        robot_desc_content = f.read()

    # 1. ROBOT İSKELET YAYINCISI
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content}]
    )

    # 2. BAĞIMSIZ BEYİN (Webots olmadan çalışan Controller Manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc_content}, controller_config],
        output='both'
    )

    # 3. KAS BAŞLATICILAR (SPAWNERS)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        jsb_spawner,
        arm_spawner
    ])