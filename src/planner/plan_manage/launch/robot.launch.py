"""
真实机器狗 + 本地地图启动：不启动随机地图与仿真，仅启动规划器并订阅机器狗双目相机与里程计话题。
用法见 docs/本地地图与机器狗相机配置说明.md
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    # ----- 通用参数 -----
    drone_id = LaunchConfiguration('drone_id', default=0)
    obj_num = LaunchConfiguration('obj_num', default=10)
    map_size_x = LaunchConfiguration('map_size_x', default=90.0)  # 原为50.0
    map_size_y = LaunchConfiguration('map_size_y', default=130.0)  # 原为25.0
    map_size_z = LaunchConfiguration('map_size_z', default=10.0)  # 原为2.0

    # ----- 真实机器狗 / 本地地图 开关 -----
    use_real_robot = LaunchConfiguration('use_real_robot', default=True)
    load_local_map = LaunchConfiguration('load_local_map', default=False)
    map_file = LaunchConfiguration('map_file', default='')

    # ----- 机器狗话题与相机参数（use_real_robot 时使用） -----
    odometry_topic = LaunchConfiguration('odometry_topic', default='/odometry')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/lidar_points')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/camera_pose')
    pose_type = LaunchConfiguration('pose_type', default=2)
    cx = LaunchConfiguration('cx', default='959.196655')
    cy = LaunchConfiguration('cy', default='538.812378')
    fx = LaunchConfiguration('fx', default='805.299072')
    fy = LaunchConfiguration('fy', default='805.879883')

    # ----- 仿真用里程计话题（use_real_robot=False 时由 simulator 使用） -----
    odom_topic_sim = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    # ----- 声明 Launch 参数 -----
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID'))
    ld.add_action(DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects'))
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size X (m)'))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size Y (m)'))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size Z (m)'))
    ld.add_action(DeclareLaunchArgument('use_real_robot', default_value=use_real_robot,
                                        description='True: no map gen, no simulator; use robot topics'))
    ld.add_action(DeclareLaunchArgument('load_local_map', default_value=load_local_map,
                                        description='True: start map loader (publish PCD to /map_generator/global_cloud)'))
    ld.add_action(DeclareLaunchArgument('map_file', default_value=map_file,
                                        description='Path to local map PCD file'))
    ld.add_action(DeclareLaunchArgument('odometry_topic', default_value=odometry_topic,
                                        description='Robot odometry topic'))
    ld.add_action(DeclareLaunchArgument('depth_topic', default_value=depth_topic,
                                        description='Depth image topic (e.g. stereo depth)'))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value=cloud_topic,
                                        description='Point cloud topic'))
    ld.add_action(DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic,
                                        description='Camera pose topic (used when pose_type=1)'))
    ld.add_action(DeclareLaunchArgument('pose_type', default_value=pose_type,
                                        description='grid_map pose_type: 1=PoseStamped, 2=Odometry'))
    ld.add_action(DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx'))
    ld.add_action(DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy'))
    ld.add_action(DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx'))
    ld.add_action(DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value=odom_topic_sim,
                                        description='Odometry topic for simulator (when use_real_robot=False)'))

    # ----- 仅仿真模式：随机地图节点（与 single_run_in_sim 一致） -----
    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    ld.add_action(DeclareLaunchArgument('use_mockamap', default_value=use_mockamap,
                                        description='Use mockamap when in sim mode'))
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    ld.add_action(DeclareLaunchArgument('use_dynamic', default_value=use_dynamic,
                                        description='Use dynamics sim when in sim mode'))

    # 仅当 非真实机器狗 时添加地图：random_forest 与 mockamap 二选一
    condition_sim_no_mock = IfCondition(PythonExpression([
        '"', use_real_robot, '" == "False" and "', use_mockamap, '" == "False"'
    ]))
    condition_sim_mock = IfCondition(PythonExpression([
        '"', use_real_robot, '" == "False" and "', use_mockamap, '" == "True"'
    ]))
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0}, {'map/y_size': 20.0}, {'map/z_size': 3.0},
            {'map/resolution': 0.1}, {'ObstacleShape/seed': 1.0}, {'map/obs_num': 250},
            {'ObstacleShape/lower_rad': 0.5}, {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0}, {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 250}, {'ObstacleShape/radius_l': 0.7}, {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7}, {'ObstacleShape/z_h': 0.8}, {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0}, {'min_distance': 0.8}
        ],
        condition=condition_sim_no_mock
    )
    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[('/mock_map', '/map_generator/global_cloud')],
        parameters=[
            {'seed': 127}, {'update_freq': 0.5}, {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1}, {'complexity': 0.05}, {'fill': 0.12}, {'fractal': 1}, {'attenuation': 0.1}
        ],
        condition=condition_sim_mock
    )
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)

    # ----- 规划器参数：真实机器狗用机器人话题，仿真用 pcl_render 话题 -----
    # 真实机器狗时传入的话题与相机内参
    advanced_param_include_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odometry_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'cx': cx, 'cy': cy, 'fx': fx, 'fy': fy,
            'pose_type': pose_type,
            'max_vel': '2.0', 'max_acc': '6.0', 'planning_horizon': '7.5',
            'use_distinctive_trajs': 'True', 'flight_type': '3',
            'plan_xy_only': 'True',
            'point_num': '4',
            'point0_x': '31.2', 'point0_y': '-6.4', 'point0_z': '1.9',
            'point1_x': '33.0', 'point1_y': '-2.5', 'point1_z': '1.9',
            'point2_x': '36.8', 'point2_y': '20.3', 'point2_z': '1.5',
            'point3_x': '39.8', 'point3_y': '41.5', 'point3_z': '1.01',
            'point4_x': '44.4', 'point4_y': '53.2', 'point4_z': '0.9',
        }.items(),
        condition=IfCondition(use_real_robot)
    )
    # 仿真时与 single_run_in_sim 一致
    advanced_param_include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic_sim,
            'obj_num_set': obj_num,
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            'cx': '321.04638671875', 'cy': '243.44969177246094',
            'fx': '387.229248046875', 'fy': '387.229248046875',
            'max_vel': '2.0', 'max_acc': '6.0', 'planning_horizon': '7.5',
            'use_distinctive_trajs': 'True', 'flight_type': '2',
            'point_num': '4',
            'point0_x': '15.0', 'point0_y': '0.0', 'point0_z': '1.0',
            'point1_x': '-15.0', 'point1_y': '0.0', 'point1_z': '1.0',
            'point2_x': '15.0', 'point2_y': '0.0', 'point2_z': '1.0',
            'point3_x': '-15.0', 'point3_y': '0.0', 'point3_z': '1.0',
            'point4_x': '15.0', 'point4_y': '0.0', 'point4_z': '1.0',
        }.items(),
        condition=UnlessCondition(use_real_robot)
    )
    ld.add_action(advanced_param_include_real)
    ld.add_action(advanced_param_include_sim)

    # ----- 轨迹伺服（仿真与真实机器狗共用） -----
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[{'traj_server/time_forward': 1.0}]
    )
    ld.add_action(traj_server_node)

    # ----- 仅仿真模式：启动 simulator（pcl_render、poscmd_2_odom、可视化等） -----
    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': '-15.0', 'init_y_': '0.0', 'init_z_': '0.1',
            'odometry_topic': odom_topic_sim
        }.items(),
        condition=UnlessCondition(use_real_robot)
    )
    ld.add_action(simulator_include)

    # ----- 可选：本地地图加载（发布 PCD 到 /map_generator/global_cloud） -----
    # 若需从 PCD 加载本地地图，可单独运行 map_loader 或发布 /map_generator/global_cloud；
    # 此处未内置 map_loader 节点，见 docs/本地地图与机器狗相机配置说明.md

    return ld
