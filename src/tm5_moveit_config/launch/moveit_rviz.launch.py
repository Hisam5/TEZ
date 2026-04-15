import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveIt ayarlarını içe aktar
    moveit_config = MoveItConfigsBuilder("tm5_900", package_name="tm5_moveit_config").to_moveit_configs()

    # RViz konfigürasyon dosyasının yolunu manuel olarak buluyoruz
    rviz_config_file = os.path.join(
        get_package_share_directory("tm5_moveit_config"),
        "config",
        "moveit.rviz"
    )

    # RViz düğümünü doğrudan biz oluşturuyoruz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}, # ⏱️ RViz'in saatini de Webots'a kilitledik!
        ],
    )

    return LaunchDescription([rviz_node])