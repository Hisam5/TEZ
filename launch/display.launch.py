import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='tm5_900').find('tm5_900')
    urdf_model_path = os.path.join(pkg_share, 'urdf', 'tm5_900.urdf')

    # Check if URDF exists
    if not os.path.exists(urdf_model_path):
        print(f"Error: URDF file not found at {urdf_model_path}")

    # Read the URDF file
    with open(urdf_model_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
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
