import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()
    # export TURTLEBOT3_MODEL ~~
    TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL', 'waffle_pi')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    config_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'params', 'robots.json')

    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    # cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
    #                                               turtlebot3_cartographer_prefix, 'config'))
    cartographer_config_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'params')
    # configuration_basename = LaunchConfiguration(cartographer_config_dir,
    #                                              default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.1')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'rec.sdf'
    )
    # config.json 로드
    with open(config_path, 'r') as f:
        robot_configs = json.load(f)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        output="screen",
        name="octomap_server",
        # namespace=namespace,
        parameters=[{
            "frame_id": "map",
            "resolution": resolution
        }],
        remappings=[
            ('cloud_in', '/agent1/scan_matched_points2'),
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static')
        ]
    )
    # 각 TurtleBot3를 위한 GroupAction 생성
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    robot_spawns = []
    first_robot = True
    for robot in robot_configs:
        namespace = ['/' + robot['name']]
        x_pose = float(robot['x_pose'])
        y_pose = float(robot['y_pose'])
        z_pose = 0.01 

        state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': str(robot['name'] + '/')
            }],
            # remappings=remappings,
        )        
        
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot['name'],  # 각 로봇 이름을 namespace로 설정
                '-file', sdf_path,
                '-x', str(x_pose), '-y', str(y_pose), '-z', str(z_pose),
                '-robot_namespace', namespace
            ],
            output='screen',
        )
        
        cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', str(f"{robot['name']}.lua")
            ],
            # remappings=remappings
            remappings=[
                ('/odom', 'odom')
            ]
        )
        if first_robot is True:

            first_robot = False
            robot_group = GroupAction([
                cartographer_node,
                spawn_robot,
                # octomap_node,
                state_publisher_node
            ])
        else:
            robot_group = GroupAction([
                cartographer_node,
                spawn_robot,
                state_publisher_node
            ])
        robot_spawns.append(robot_group)

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        octomap_node,
    ] + robot_spawns)
