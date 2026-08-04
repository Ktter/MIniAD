from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    kinematic_world = PathJoinSubstitution([FindPackageShare('ad_gazebo_sim'), 'worlds', 'minimal.world'])
    ackermann_world = PathJoinSubstitution([FindPackageShare('ad_gazebo_sim'), 'worlds', 'ackermann.world'])
    models = PathJoinSubstitution([FindPackageShare('ad_gazebo_sim'), 'models'])
    kinematic_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])),
        launch_arguments={
            'world': kinematic_world,
            'verbose': 'true',
            'pause': 'false',
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'kinematic'"])))
    ackermann_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])),
        launch_arguments={
            'world': ackermann_world,
            'verbose': 'true',
            'pause': 'false',
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ackermann_physics'"])))
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='acc'),
        DeclareLaunchArgument('mode', default_value='kinematic'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        kinematic_gazebo,
        ackermann_gazebo,
        Node(package='ad_gazebo_sim', executable='gazebo_control_adapter', output='screen'),
        Node(package='ad_gazebo_sim', executable='gazebo_state_adapter', output='screen'),
    ])
