from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = PathJoinSubstitution([FindPackageShare('putn'), 'config', 'for_real_scenarios', 'general.yaml'])

    pcd_path_arg = DeclareLaunchArgument('pcd_path', default_value='')
    crop_enabled_arg = DeclareLaunchArgument('crop_enabled', default_value='true')
    crop_min_x_arg = DeclareLaunchArgument('crop_min_x', default_value='-10.0')
    crop_max_x_arg = DeclareLaunchArgument('crop_max_x', default_value='10.0')
    crop_min_y_arg = DeclareLaunchArgument('crop_min_y', default_value='-4.0')
    crop_max_y_arg = DeclareLaunchArgument('crop_max_y', default_value='10.0')

    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    global_planning = Node(
        package='putn',
        executable='global_planning_node',
        name='global_planning_node',
        output='screen',
        parameters=[param_file],
        remappings=[
            ('map', '/test_map'),
            ('waypoints', '/waypoints'),
        ],
    )

    gpr_path = Node(
        package='gpr',
        executable='gpr_path',
        name='gpr_path',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/global_planning_node/global_path', '/global_path'),
            ('/global_planning_node/tree_tra', '/tree_tra'),
            ('/surf_predict_pub', '/surf_predict_pub'),
        ],
    )

    waypoint_gen = Node(
        package='waypoint_generator',
        executable='waypoint_generator',
        name='waypoint_generator',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('goal', '/goal'),
            ('odom', '/odom')
        ],
    )

    pcd_publisher = Node(
        package='local_planner',
        executable='pcd_publisher.py',
        name='pcd_publisher',
        output='screen',
        parameters=[
            {'frame_id': 'map'},
            {'topic': '/test_map'},
            {'repeat_rate': 0.1},
            {'pcd_path': LaunchConfiguration('pcd_path')},
            {'crop/enabled': LaunchConfiguration('crop_enabled')},
            {'crop/min_x': LaunchConfiguration('crop_min_x')},
            {'crop/max_x': LaunchConfiguration('crop_max_x')},
            {'crop/min_y': LaunchConfiguration('crop_min_y')},
            {'crop/max_y': LaunchConfiguration('crop_max_y')},
        ],
    )

    fake_odom = Node(
        package='local_planner',
        executable='fake_odom_publisher.py',
        name='fake_odom',
        output='screen',
        parameters=[{'frame_id': 'world'}, {'child_frame_id': 'base_link'}, {'x': 0.0}, {'y': 0.0}, {'z': 0.0}, {'yaw': 0.0}, {'rate': 30.0}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'src/putn_ROS2/src/putn/putn_launch/rviz/test.rviz'],
        output='screen'
    )

    return LaunchDescription([
        pcd_path_arg,
        crop_enabled_arg,
        crop_min_x_arg,
        crop_max_x_arg,
        crop_min_y_arg,
        crop_max_y_arg,
        world_to_map,
        pcd_publisher,
        fake_odom,
        waypoint_gen,
        global_planning,
        gpr_path,
        rviz2,
    ])
