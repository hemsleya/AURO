import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace, RosTimer


def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    yaml_path = os.path.join(get_package_share_directory(context.launch_configurations['initial_pose_package']), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    actions = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                # prefix=['xfce4-terminal --tab --execute'], # Opens in new tab
                # prefix=['xfce4-terminal --execute'], # Opens in new window
                # prefix=['gnome-terminal --tab --execute'], # Opens in new tab
                # prefix=['gnome-terminal --window --execute'], # Opens in new window
                # prefix=['wt.exe --window 0 new-tab wsl.exe -e bash -ic'], # Opens in new tab
                # prefix=['wt.exe wsl.exe -e bash -ic'], # Opens in new window
                output='screen',
                parameters=[initial_poses[robot_name]]),

            # Node(
            #     package='turtlebot3_gazebo',
            #     executable='turtlebot3_drive',
            #     output='screen'),

        ])

        actions.append(group)

    return actions

def generate_launch_description():

    package_name = 'solution'

    num_robots = LaunchConfiguration('num_robots')
    headless = LaunchConfiguration('headless')
    obstacles = LaunchConfiguration('obstacles')
    zone_top_left = LaunchConfiguration('zone_top_left')
    sensor_noise = LaunchConfiguration('sensor_noise')
    initial_pose_package = LaunchConfiguration('initial_pose_package')
    initial_pose_file = LaunchConfiguration('initial_pose_file')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')
    data_log_path = LaunchConfiguration('data_log_path')
    data_log_filename = LaunchConfiguration('data_log_filename')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run simulation in headless mode')
    
    declare_obstacles_cmd = DeclareLaunchArgument(
        'obstacles',
        default_value='True',
        description='Run simulation without obstacles')
    
    declare_zone_top_left_cmd = DeclareLaunchArgument(
        'zone_top_left',
        default_value='True',
        description='Activate zone top left')
    
    declare_sensor_noise_cmd = DeclareLaunchArgument(
        'sensor_noise',
        default_value='False',
        description='Run simulation with sensor noise')
    
    declare_initial_pose_package_cmd = DeclareLaunchArgument(
        'initial_pose_package',
        default_value='assessment',
        description='Set package name for initial pose file')
    
    declare_initial_pose_file_cmd = DeclareLaunchArgument(
        'initial_pose_file',
        default_value='config/initial_poses.yaml',
        description='Set filename for initial pose file')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager')
    
    declare_experiment_duration_cmd = DeclareLaunchArgument(
        'experiment_duration',
        default_value='300.0',
        description='Experiment duration in seconds')
    
    declare_data_log_path_cmd = DeclareLaunchArgument(
        'data_log_path',
        default_value = os.path.join(get_package_prefix(package_name), '../../'),
        description='Full path to directory where data logs will be saved')
    
    declare_data_log_filename_cmd = DeclareLaunchArgument(
        'data_log_filename',
        default_value='data_log',
        description='Filename prefix to use for data logs')
    

    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced_nav2.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare('assessment'), 'config', 'rviz_windows.yaml'])
    # rviz_windows = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'custom_rviz_windows.yaml'])
    map = PathJoinSubstitution([FindPackageShare('assessment'), 'maps', 'assessment_world.yaml'])
    #params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])
    params = PathJoinSubstitution([FindPackageShare(package_name), 'params', 'custom_nav2_params_namespaced.yaml'])


    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    #launch global map_server so all robots share 1 map
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map,
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': num_robots,
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'rviz_config': rviz_config,
                          'rviz_windows': rviz_windows,
                          'obstacles': obstacles,
                          'zone_top_left': zone_top_left,
                          'item_manager': 'true',
                          'random_seed': random_seed,
                          'use_nav2': 'True',
                          'map': map,
                          'map_server': 'False',
                          'params_file': params,
                          'headless': headless,
                          'limit_real_time_factor': 'true',
                          'wait_for_items': 'false',
                          'sensor_noise': sensor_noise,
                          'initial_pose_package': initial_pose_package,
                          'initial_pose_file': initial_pose_file,
                          # 'extra_gazebo_args': '--verbose',
                          }.items()
    )

    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    data_logger_cmd = Node(
        package='solution',
        executable='data_logger',
        output='screen',
        arguments=['--path', data_log_path,
                   '--filename', data_log_filename,
                   '--random_seed', random_seed])

    timeout_cmd = RosTimer(                                         
            period = experiment_duration,
            actions = [                                                       
                Shutdown(reason="Experiment timeout reached")     
            ],
        )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)

    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_obstacles_cmd)
    ld.add_action(declare_zone_top_left_cmd)
    ld.add_action(declare_sensor_noise_cmd)
    ld.add_action(declare_initial_pose_package_cmd)
    ld.add_action(declare_initial_pose_file_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_experiment_duration_cmd)
    ld.add_action(declare_data_log_path_cmd)
    ld.add_action(declare_data_log_filename_cmd)

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(data_logger_cmd)
    ld.add_action(timeout_cmd)

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)


    return ld
