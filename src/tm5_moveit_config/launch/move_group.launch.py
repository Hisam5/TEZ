from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveIt ayarlarını içe aktar (Senin yazdığın kısım)
    moveit_config = MoveItConfigsBuilder("tm5_900", package_name="tm5_moveit_config").to_moveit_configs()

    # Kapalı generate_move_group_launch fonksiyonu yerine doğrudan Node oluşturuyoruz
    # Bu sayede MoveIt'in beynine kendi saat kurallarımızı zorla yazabiliyoruz
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},  # ⏱️ 56 yıllık zaman farkı sorununu çözen sihirli satır!
        ],
    )

    return LaunchDescription([move_group_node])