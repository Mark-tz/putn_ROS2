# /home/turingzero/ros_ws/putn/src/putn/putn_launch/launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    scene_arg = DeclareLaunchArgument('scene', default_value='map1')
    param_file = PathJoinSubstitution([
        FindPackageShare('putn'),
        'config',
        'for_simulation',
        PythonExpression(["'", LaunchConfiguration('scene'), "'", " + '.yaml'"])
    ])
    rviz_config = PathJoinSubstitution([FindPackageShare('putn_launch'), 'rviz_config', 'simulation.rviz'])

    world_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_init']
    )
    camera_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_link',
        arguments=['0', '0', '-0.4', '0', '0', '0', 'aft_mapped', 'base_link']
    )

    global_planning = Node(
        package='putn',
        executable='global_planning_node',
        name='global_planning_node',
        output='screen',
        parameters=[param_file],
        remappings=[
            ('map', '/laser_cloud_map'),
            ('waypoints', '/waypoint_generator/waypoints'),
        ],
    )

    local_obs = Node(
        package='putn',
        executable='local_obs_node',
        name='local_obs_node',
        output='screen',
        parameters=[{
            'map/resolution': 0.1,
            'map/local_x_l': -1.8,
            'map/local_x_u': 1.8,
            'map/local_y_l': -1.8,
            'map/local_y_u': 1.8,
            'map/local_z_l': -0.5,
            'map/local_z_u': 0.4,
        }],
        remappings=[
            ('map', '/velodyne_points'),
        ],
    )

    waypoint_gen = Node(
        package='waypoint_generator',
        executable='waypoint_generator',
        name='waypoint_generator',
        output='screen',
        parameters=[{'waypoint_type': 'manual-lonely-waypoint'}],
        remappings=[('goal', '/goal')],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        scene_arg,
        world_to_camera_init,
        camera_to_link,
        waypoint_gen,
        global_planning,
        local_obs,
        rviz2
    ])