# /home/turingzero/ros_ws/putn/src/putn/putn_launch/launch/bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file = PathJoinSubstitution([FindPackageShare('putn'), 'config', 'for_real_scenarios', 'general.yaml'])

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
            ('map', '/slam_node/map_global'),
            ('waypoints', '/waypoints'),
        ],
    )

    local_obs = Node(
        package='putn',
        executable='local_obs_node',
        name='local_obs_node',
        output='screen',
        parameters=[{
            'map/resolution_xy': 0.1,
            'map/resolution_z': 0.05,
            'map/local_x_l': -1.8,
            'map/local_x_u': 1.8,
            'map/local_y_l': -1.8,
            'map/local_y_u': 1.8,
            'map/local_z_l': -0.5,
            'map/local_z_u': 0.4,
        }],
        remappings=[
            ('map', '/slam_node/cloud_registered'),
        ],
    )

    waypoint_gen = Node(
        package='waypoint_generator',
        executable='waypoint_generator',
        name='waypoint_generator',
        output='screen',
        prefix='xterm -hold -e',
        emulate_tty=True,
        remappings=[
            ('goal', '/goal'),
            ('odom', '/odom')
        ],
    )

    gpr_path = Node(
        package='gpr',
        executable='gpr_path',
        name='gpr_path',
        output='screen',
        prefix='xterm -hold -e',
        emulate_tty=True,
        remappings=[
            ('/global_planning_node/global_path', '/global_path'),
            ('/global_planning_node/tree_tra', '/tree_tra'),
            ('/surf_predict_pub', '/surf_predict_pub'),
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'src/putn/src/putn/putn_launch/rviz/rviz.rviz'],
        output='screen'
    )

    local_planner = Node(
        package='local_planner',
        executable='local_planner.py',
        name='local_planner',
        output='screen',
        prefix='xterm -hold -e',
        emulate_tty=True
    )

    controller = Node(
        package='local_planner',
        executable='controller.py',
        name='controller',
        output='screen',
        prefix='xterm -hold -e',
        emulate_tty=True
    )

    return LaunchDescription([
        world_to_map,
        waypoint_gen,
        gpr_path,
        global_planning,
        local_obs,
        rviz2,
        local_planner,
        controller
    ])
