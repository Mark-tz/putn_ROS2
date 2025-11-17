# /home/turingzero/ros_ws/putn/src/putn/putn_map/launch/start.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value=PathJoinSubstitution([FindPackageShare('putn_map'), 'worlds', 'empty.world']))
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    return LaunchDescription([world_arg, gui_arg, gazebo_launch])