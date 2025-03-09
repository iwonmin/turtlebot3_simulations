import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # TURTLEBOT3_MODEL 환경 변수 처리
    TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL', 'waffle_pi')  # 기본값 'waffle' 설정
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
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration(cartographer_config_dir,
                                                 default='turtlebot3_lds_2d.lua')
    
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

    # 각 TurtleBot3를 위한 GroupAction 생성
    robot_spawns = []
    for robot in robot_configs:
        namespace = robot['name']
        x_pose = float(robot['x_pose'])  # 명시적으로 float 형 변환
        y_pose = float(robot['y_pose'])  # 명시적으로 float 형 변환
        z_pose = 0.01  # 기본 z값 설정

        # 로봇 스폰
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', namespace,  # 각 로봇 이름을 namespace로 설정
                '-file', sdf_path,
                '-x', str(x_pose), '-y', str(y_pose), '-z', str(z_pose),
                '-robot_namespace', f"/{namespace}"
            ],
            output='screen',
            remappings=[
                # ("/tf", "tf"),
                # ("/tf_static", "tf_static")
            ]
        )
        octomap_node = Node(
            package="octomap_server",
            executable="octomap_server_node",
            output="screen",
            name="octomap_server",
            namespace=f"/{namespace}",
            parameters=[
                {"frame_id": "map",
                "resolution": 0.1}
            ],
            remappings=[
                ('cloud_in', f'/{namespace}'+'/scan_matched_points2'),
                # ("/tf", "tf"),
                # ("/tf_static", "tf_static")
            ]
        )
        cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=f"/{namespace}",
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # 'tracking_frame': f'/{namespace}/base_link',
                # 'published_frame': f'/{namespace}/odom',
                # 'imu_frame': f'/{namespace}/imu_link',
                # 'map_frame':f'/{namespace}/map'
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                # ("/tf", "tf"),
                # ("/tf_static", "tf_static")
            ]
        )
        state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=f"/{namespace}",
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_desc,
                    'frame_prefix': f'{namespace}/'
                }],
                remappings=[
                    # ("/tf", "tf"),
                    # ("/tf_static", "tf_static")
                ]
            )
                       
        # 로봇의 센서 및 드라이브 노드 실행
        robot_group = GroupAction([
            cartographer_node,
            spawn_robot,
            octomap_node,
            state_publisher_node
        ])
        robot_spawns.append(robot_group)

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
    ] + robot_spawns)
