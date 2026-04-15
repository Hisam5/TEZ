import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paket dizinlerini buluyoruz (Farklı paketlerde olduklarını belirttiğin için)
    tm5_900_dir = get_package_share_directory('tm5_900')
    moveit_config_dir = get_package_share_directory('tm5_moveit_config')

    # 1. Gövde ve Kaslar: Kendi yazdığın o kusursuz Webots dosyasını çağırıyoruz
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tm5_900_dir, 'launch', 'webotss.launch.py')
        )
    )

    # 2. Beyin: MoveIt Planlayıcısını çağırıyoruz
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
        )
    )

    # 3. Gözler: RViz'i çağırıyoruz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # ZAMANLAMALAR (Kritik Nokta)
    # Webots'un dünyayı yüklemesi, robotu oluşturması ve ros2_control'ü ayağa kaldırması zaman alır.
    # MoveIt beyni, Webots hazır olmadan açılırsa motorları bulamayıp çöker.
    # Bu yüzden beyne ve gözlere birkaç saniyelik gecikme (delay) ekliyoruz.
    
    delayed_move_group = TimerAction(period=4.0, actions=[move_group_launch])
    delayed_rviz = TimerAction(period=6.0, actions=[rviz_launch])

    # Hepsini paketleyip sisteme fırlatıyoruz
    return LaunchDescription([
        webots_launch,
        delayed_move_group,
        delayed_rviz
    ])