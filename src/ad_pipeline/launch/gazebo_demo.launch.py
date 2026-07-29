from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ad_gazebo_sim'),'launch','gazebo.launch.py'])), launch_arguments={'scenario': scenario}.items())
    return LaunchDescription([DeclareLaunchArgument('scenario', default_value='acc'), sim, Node(package='ad_localization', executable='localization_node', output='screen'), Node(package='ad_perception', executable='perception_node', output='screen'), Node(package='ad_behavior', executable='behavior_node', parameters=[{'scenario': scenario}], output='screen'), Node(package='ad_planning_control', executable='planning_control_node', output='screen')])
