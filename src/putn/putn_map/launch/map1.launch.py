# /home/turingzero/ros_ws/putn/src/putn/putn_map/launch/map1.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('putn_map'), 'launch', 'start.launch.py'])),
            launch_arguments={
                'world': PathJoinSubstitution([FindPackageShare('putn_map'), 'worlds', 'map1.world'])
            }.items()
        )
    ])