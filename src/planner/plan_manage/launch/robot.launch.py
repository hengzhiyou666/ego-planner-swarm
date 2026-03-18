"""
真实机器狗启动：仅启动 ego_planner（规划器），订阅机器狗传感器/里程计话题。
用法见 docs/本地地图与机器狗相机配置说明.md
可通过 rviz:=false 关闭自动启动 RViz。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # =================先记住“我要用哪个参数”，等真正启动时再把实际值塞进来==================
    # ----- 通用参数（dog_id：机器狗 ID） -----
    dog_id = LaunchConfiguration('dog_id', default=0)
    obj_num = LaunchConfiguration('obj_num', default=10)
    map_size_x = LaunchConfiguration('map_size_x', default=90.0)  # 原为50.0
    map_size_y = LaunchConfiguration('map_size_y', default=130.0)  # 原为25.0
    map_size_z = LaunchConfiguration('map_size_z', default=10.0)  # 原为2.0

    # 调试模式：沿全局路径仅向前走指定米数；-1.0 表示关闭调试，正常使用完整 /pct_path
    debugMode_testGoForward_m = LaunchConfiguration('debugMode_testGoForward_m', default='-1.0')

    # ----- 真实机器狗开关（本工程清理后仅支持 True；保留参数避免用户脚本报错） -----
    use_real_robot = LaunchConfiguration('use_real_robot', default=True)

    # ----- 机器狗话题与相机参数（use_real_robot 时使用） -----
    odometry_topic = LaunchConfiguration('odometry_topic', default='/odometry')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/lidar_points')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/camera_pose')
    input_pose_message_type = LaunchConfiguration('input_pose_message_type', default=1)
    frame_id = LaunchConfiguration('frame_id', default='head_init')
    enable_realtime_occupancy_grid = LaunchConfiguration('enable_realtime_occupancy_grid', default='1')
    rviz = LaunchConfiguration('rviz', default='true')
    pct_path_skip_if_same = LaunchConfiguration('pct_path_skip_if_same', default=True)
    cx = LaunchConfiguration('cx', default='959.196655')
    cy = LaunchConfiguration('cy', default='538.812378')
    fx = LaunchConfiguration('fx', default='805.299072')
    fy = LaunchConfiguration('fy', default='805.879883')

    # ----- 规划器固定参数（原 advanced_param 内嵌，保持默认行为一致） -----
    max_vel = LaunchConfiguration('max_vel', default='2.0')
    max_acc = LaunchConfiguration('max_acc', default='1.0')
    path_ahead_time = LaunchConfiguration('path_ahead_time', default='7.5')
    try_more_paths_and_choose_best = LaunchConfiguration('try_more_paths_and_choose_best', default='True')
    egoplanner_input_point_or_path = LaunchConfiguration('egoplanner_input_point_or_path', default='3')
    plan_xy_only = LaunchConfiguration('plan_xy_only', default='False')
    point_num = LaunchConfiguration('point_num', default='4')
    point0_x = LaunchConfiguration('point0_x', default='31.2')
    point0_y = LaunchConfiguration('point0_y', default='-6.4')
    point0_z = LaunchConfiguration('point0_z', default='1.9')
    point1_x = LaunchConfiguration('point1_x', default='33.0')
    point1_y = LaunchConfiguration('point1_y', default='-2.5')
    point1_z = LaunchConfiguration('point1_z', default='1.9')
    point2_x = LaunchConfiguration('point2_x', default='36.8')
    point2_y = LaunchConfiguration('point2_y', default='20.3')
    point2_z = LaunchConfiguration('point2_z', default='1.5')
    point3_x = LaunchConfiguration('point3_x', default='39.8')
    point3_y = LaunchConfiguration('point3_y', default='41.5')
    point3_z = LaunchConfiguration('point3_z', default='1.01')
    point4_x = LaunchConfiguration('point4_x', default='44.4')
    point4_y = LaunchConfiguration('point4_y', default='53.2')
    point4_z = LaunchConfiguration('point4_z', default='0.9')

    # ----- 声明 Launch 参数 -----
    launch_plan = LaunchDescription()
    launch_plan.add_action(DeclareLaunchArgument('dog_id', default_value='0', description='Dog ID'))
    launch_plan.add_action(DeclareLaunchArgument('obj_num', default_value='10', description='Number of objects'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_x', default_value='90.0', description='Map size X (m)'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_y', default_value='130.0', description='Map size Y (m)'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_z', default_value='10.0', description='Map size Z (m)'))
    launch_plan.add_action(DeclareLaunchArgument(
        'debugMode_testGoForward_m',
        default_value='-1.0',
        description='调试模式：>0 仅使用从起点向前指定米数的 /pct_path；-1.0 关闭调试，使用完整全局路径（类型为 double）'
    ))
    launch_plan.add_action(DeclareLaunchArgument('use_real_robot', default_value='true',
                                                description='True: no map gen, no simulator; use robot topics'))
    launch_plan.add_action(DeclareLaunchArgument('odometry_topic', default_value='/odometry', description='Robot odometry topic'))
    launch_plan.add_action(DeclareLaunchArgument('depth_topic', default_value='/depth', description='Depth image topic (e.g. stereo depth)'))
    launch_plan.add_action(DeclareLaunchArgument('cloud_topic', default_value='/lidar_points', description='Point cloud topic'))
    launch_plan.add_action(DeclareLaunchArgument('camera_pose_topic', default_value='/camera_pose', description='Camera pose topic (used when input_pose_message_type=1)'))
    launch_plan.add_action(DeclareLaunchArgument('input_pose_message_type', default_value='1',
                                                description='grid_map 输入位姿话题消息类型: 1=PoseStamped, 2=Odometry'))
    launch_plan.add_action(DeclareLaunchArgument('frame_id', default_value='head_init', description='Planning/map frame id (odom/map)'))
    launch_plan.add_action(DeclareLaunchArgument(
        'enable_realtime_occupancy_grid', default_value='1',
        description='Whether to publish occupancy grid pointcloud in realtime (1=on,0=off)'))
    launch_plan.add_action(DeclareLaunchArgument('cx', default_value='959.196655', description='Camera intrinsic cx'))
    launch_plan.add_action(DeclareLaunchArgument('cy', default_value='538.812378', description='Camera intrinsic cy'))
    launch_plan.add_action(DeclareLaunchArgument('fx', default_value='805.299072', description='Camera intrinsic fx'))
    launch_plan.add_action(DeclareLaunchArgument('fy', default_value='805.879883', description='Camera intrinsic fy'))
    launch_plan.add_action(DeclareLaunchArgument('rviz', default_value='true',
                                                description='Whether to auto-start RViz (config: Fixed Frame head_init, topics pct_path_unfinished, odometry, plan_vis)'))
    launch_plan.add_action(DeclareLaunchArgument('pct_path_skip_if_same', default_value='false',
                                                description='True: skip when /pct_path same as last; False: no check, always execute'))

    # ----- EGO Planner 节点（原 advanced_param.launch.py 内联，仅 use_real_robot 时启动） -----
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['dog_', dog_id, '_ego_planner_node'],
        output='screen',
        condition=IfCondition(use_real_robot),
        remappings=[
            ('odom_world', odometry_topic),
            ('planning/bspline', ['dog_', dog_id, '_planning/bspline']),
            ('planning/data_display', ['dog_', dog_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('goal_point', ['dog_', dog_id, '_plan_vis/goal_point']),
            ('global_list', ['dog_', dog_id, '_plan_vis/global_list']),
            ('init_list', ['dog_', dog_id, '_plan_vis/init_list']),
            ('optimal_list', ['dog_', dog_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['dog_', dog_id, '_plan_vis/a_star_list']),
            ('grid_map/odom', odometry_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/pose', camera_pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', ['dog_', dog_id, '_grid/grid_map/occupancy_inflate'])
        ],
        parameters=[
            {'fsm/egoplanner_input_point_or_path': egoplanner_input_point_or_path},
            {'fsm/thresh_replan_time': 3.0},
            {'fsm/thresh_no_replan_meter': 0.5},
            {'fsm/path_ahead_time': path_ahead_time},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            {'fsm/plan_xy_only': plan_xy_only},
            {'fsm/pct_path_skip_if_same': pct_path_skip_if_same},
            {'fsm/debugMode_testGoForward_m': debugMode_testGoForward_m},
            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x}, {'fsm/waypoint0_y': point0_y}, {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x}, {'fsm/waypoint1_y': point1_y}, {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x}, {'fsm/waypoint2_y': point2_y}, {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x}, {'fsm/waypoint3_y': point3_y}, {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x}, {'fsm/waypoint4_y': point4_y}, {'fsm/waypoint4_z': point4_z},
            {'grid_map/resolution': 0.2},
            {'grid_map/map_size_x': map_size_x}, {'grid_map/map_size_y': map_size_y}, {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5}, {'grid_map/local_update_range_y': 5.5}, {'grid_map/local_update_range_z': 1.5},
            {'grid_map/obstacles_inflation': 0.099}, {'grid_map/local_map_margin': 10}, {'grid_map/ground_height': 0.2},
            {'grid_map/use_depth_for_occupancy': False},
            {'grid_map/cx': cx}, {'grid_map/cy': cy}, {'grid_map/fx': fx}, {'grid_map/fy': fy},
            {'grid_map/use_depth_filter': True}, {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0}, {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 4}, {'grid_map/k_depth_scaling_factor': 1.0}, {'grid_map/skip_pixel': 4},
            {'grid_map/p_hit': 0.65}, {'grid_map/p_miss': 0.35}, {'grid_map/p_min': 0.12}, {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80}, {'grid_map/min_ray_length': 0.1}, {'grid_map/max_ray_length': 4.5},
            {'grid_map/virtual_ceil_height': 1.0}, {'grid_map/visualization_truncate_height': 2.5},
            {'grid_map/show_occ_time': False},
            {'grid_map/enable_realtime_occupancy_vis': enable_realtime_occupancy_grid},
            {'grid_map/input_pose_message_type': input_pose_message_type}, {'grid_map/frame_id': frame_id},
            {'manager/max_vel': max_vel}, {'manager/max_acc': max_acc}, {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.1}, {'manager/feasibility_tolerance': 0.05},
            {'manager/path_ahead_time': path_ahead_time},
            {'manager/try_more_paths_and_choose_best': try_more_paths_and_choose_best},
            {'manager/dog_id': dog_id},
            {'optimization/lambda_smooth': 1.0}, {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1}, {'optimization/lambda_fitness': 5.0},
            {'optimization/dist0': 0.5}, {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel}, {'optimization/max_acc': max_acc},
            {'bspline/limit_vel': max_vel}, {'bspline/limit_acc': max_acc}, {'bspline/limit_ratio': 1.1},
            {'prediction/obj_num': obj_num}, {'prediction/lambda': 1.0}, {'prediction/predict_rate': 1.0}
        ]
    )
    launch_plan.add_action(ego_planner_node)

    # ----- 可选：仅当 rviz 为 true 时自动启动 RViz（Fixed Frame: head_init；话题见 config/robot.rviz） -----
    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'config', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )
    launch_plan.add_action(GroupAction([rviz_node], condition=IfCondition(rviz)))

    return launch_plan
