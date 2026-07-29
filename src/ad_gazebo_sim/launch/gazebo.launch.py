from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='acc'),
        Node(package='ad_gazebo_sim', executable='cpu_simulator', name='cpu_simulator', output='screen', parameters=[{'scenario': LaunchConfiguration('scenario')}]),
    ])
