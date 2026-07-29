from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='acc'),
        Node(package='ad_gazebo_sim', executable='cpu_simulator', parameters=[{'scenario': scenario}], output='screen'),
        Node(package='ad_localization', executable='localization_node', output='screen'),
        Node(package='ad_perception', executable='perception_node', output='screen'),
        Node(package='ad_behavior', executable='behavior_node', parameters=[{'scenario': scenario}], output='screen'),
        Node(package='ad_planning_control', executable='planning_control_node', output='screen'),
    ])
