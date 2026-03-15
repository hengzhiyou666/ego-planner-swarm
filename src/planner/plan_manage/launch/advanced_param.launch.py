import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    #======================告诉系统我要使用哪些变量，默认值是多少=================================================================
    # LaunchConfigurations
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odometry')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth')
    cloud_topic = LaunchConfiguration('cloud_topic', default='lidar_points')

    frame_id = LaunchConfiguration('frame_id', default='odom')
    
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    path_ahead_time = LaunchConfiguration('path_ahead_time', default=7.5)
    
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    egoplanner_input_point_or_path = LaunchConfiguration('egoplanner_input_point_or_path', default=3)#输入模式：单点 / 预设点 / 参考路径
    try_more_paths_and_choose_best = LaunchConfiguration('try_more_paths_and_choose_best', default=True)
    plan_xy_only = LaunchConfiguration('plan_xy_only', default=True)
    pct_path_skip_if_same = LaunchConfiguration('pct_path_skip_if_same', default=False)  # True：与上次路径相同时跳过计算；False：不判重，每次都执行
    
    num_of_dynamic_objects = LaunchConfiguration('num_of_dynamic_objects', default=10)
    
    drone_id = LaunchConfiguration('drone_id', default=0)
    input_pose_message_type = LaunchConfiguration('input_pose_message_type', default=2)  # 输入的位姿话题消息类型：1=PoseStamped，2=Odometry

    #=======================================================================================

    #=======================================================================================
    # DeclareLaunchArguments
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value=frame_id, description='Planning/map frame id (odom/map)')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    path_ahead_time_arg = DeclareLaunchArgument('path_ahead_time', default_value=path_ahead_time, description='规划时向前看的时间长度（秒）')
    
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    
    egoplanner_input_point_or_path_arg = DeclareLaunchArgument(
        'egoplanner_input_point_or_path',
        default_value=egoplanner_input_point_or_path,
        description='EGO Planner 输入是“单点 / 预设点 / 路径”等模式')
    try_more_paths_and_choose_best_arg = DeclareLaunchArgument(
        'try_more_paths_and_choose_best',
        default_value=try_more_paths_and_choose_best,
        description='是否尝试多条不同路径并从中挑选一条最优路径')
    plan_xy_only_arg = DeclareLaunchArgument('plan_xy_only', default_value=plan_xy_only, description='Plan in XY only, force z=0 (e.g. for robot dog)')
    pct_path_skip_if_same_arg = DeclareLaunchArgument(
        'pct_path_skip_if_same',
        default_value=pct_path_skip_if_same,
        description='True: skip computation when /pct_path same as last; False: no check, always execute')
    num_of_dynamic_objects_arg = DeclareLaunchArgument(
        'num_of_dynamic_objects',
        default_value=num_of_dynamic_objects,
        description='场景中要考虑的动态物体（障碍物）个数')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    input_pose_message_type_arg = DeclareLaunchArgument(
        'input_pose_message_type',
        default_value=input_pose_message_type,
        description='grid_map 输入位姿话题消息类型: 1=PoseStamped, 2=Odometry')

    #=======================================================================================
    # Ego Planner Node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', odometry_topic),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            
            ('grid_map/odom', odometry_topic),
            # 真实机器人常用绝对话题名（如 /lidar_points /depth /camera_pose），不应再拼接 drone_id 前缀
            ('grid_map/cloud', cloud_topic),
            ('grid_map/pose', camera_pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
parameters=[
            # ========== 一、FSM / 任务相关参数 ==========
            {'fsm/egoplanner_input_point_or_path': egoplanner_input_point_or_path},  # 目标输入模式：单点 / 航路点 / 参考路径
            {'fsm/thresh_replan_time': 3.0},                 # 触发时间重规划的时间阈值（s）
            {'fsm/thresh_no_replan_meter': 0.5},             # 若位移小于该值则不触发重规划（m）
            {'fsm/path_ahead_time': path_ahead_time},        # FSM 规划向前看的时间（s），与 manager/path_ahead_time 对齐
            {'fsm/planning_horizen_time': 3.0},              # 规划时考虑的时间地平线（s）
            {'fsm/emergency_time': 1.0},                     # 紧急停止触发的时间阈值（s）
            {'fsm/realworld_experiment': False},             # 是否为真实环境实验模式（影响安全策略）
            {'fsm/fail_safe': True},                         # 是否启用失败保护（规划失败时进入 EMERGENCY_STOP 等）
            {'fsm/plan_xy_only': plan_xy_only},              # 只在 XY 平面规划（机器狗等地面机器人）
            {'fsm/pct_path_skip_if_same': pct_path_skip_if_same},  # True：与上次路径相同时跳过计算；False：不判重，每次都执行

            # 预设航路点（当目标模式为 PRESET_TARGET 时使用）
            {'fsm/waypoint_num': point_num},                 # 航路点数量
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},

            # ========== 二、局部栅格地图 grid_map 基本尺寸与分辨率 ==========
            {'grid_map/resolution': 0.1},                    # 栅格分辨率（m）
            {'grid_map/map_size_x': map_size_x},             # 地图尺寸 X 方向（m）
            {'grid_map/map_size_y': map_size_y},             # 地图尺寸 Y 方向（m）
            {'grid_map/map_size_z': map_size_z},             # 地图尺寸 Z 方向（m）
            {'grid_map/local_update_range_x': 5.5},          # 局部更新范围 X（m）
            {'grid_map/local_update_range_y': 5.5},          # 局部更新范围 Y（m）
            {'grid_map/local_update_range_z': 4.5},          # 局部更新范围 Z（m）
            {'grid_map/obstacles_inflation': 0.099},         # 障碍膨胀半径，太大容易将机器人包进障碍中
            {'grid_map/local_map_margin': 10},               # 栅格边界预留 margin（格数）
            {'grid_map/ground_height': -5.0},                # 地面高度 z 值（以下视为地面/不可行）
            {'grid_map/use_depth_for_occupancy': False},      # False=仅点云 /lidar_points 避障；True=深度图+点云共同更新占据

            # ========== 三、相机内参（用于深度/点云投影） ==========
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},

            # ========== 四、深度图过滤参数（去噪、稀疏采样） ==========
            {'grid_map/use_depth_filter': True},             # 是否启用深度滤波
            {'grid_map/depth_filter_tolerance': 0.15},       # 深度一致性容差（m）
            {'grid_map/depth_filter_maxdist': 5.0},          # 最大深度有效距离（m）
            {'grid_map/depth_filter_mindist': 0.2},          # 最小深度有效距离（m）
            {'grid_map/depth_filter_margin': 4},             # 原为2，深度滤波时的像素邻域大小
            {'grid_map/k_depth_scaling_factor': 1.0},        # 深度值缩放系数（原为 1000）
            {'grid_map/skip_pixel': 4},                      # 原为2，每隔多少像素取一个深度点（减少计算量）

            # ========== 五、局部概率融合（占据栅格更新参数） ==========
            {'grid_map/p_hit': 0.65},                        # 观测为占据时的概率（命中）
            {'grid_map/p_miss': 0.35},                       # 观测为空闲时的概率（脱靶）
            {'grid_map/p_min': 0.12},                        # 概率下限
            {'grid_map/p_max': 0.90},                        # 概率上限
            {'grid_map/p_occ': 0.80},                        # 判定为障碍的占据概率阈值
            {'grid_map/min_ray_length': 0.1},                # 射线最小长度（m）
            {'grid_map/max_ray_length': 4.5},                # 射线最大长度（m）

            # ========== 六、可视化 & 坐标系设置 ==========
            {'grid_map/virtual_ceil_height': 2.9},           # 虚拟天花板高度（m），限制飞行高度
            {'grid_map/visualization_truncate_height': 2.5}, # 原为1.8，可视化截断显示高度
            {'grid_map/show_occ_time': False},               # 是否显示每个栅格的更新时间
            {'grid_map/input_pose_message_type': input_pose_message_type},  # 输入位姿消息类型（1=PoseStamped,2=Odometry）
            {'grid_map/frame_id': frame_id},                 # 地图/规划坐标系（如 odom/map）

            # ========== 七、规划器管理与优化参数 ==========
            # ========== 轨迹管理相关参数（位置/速度/采样密度等） ==========
            {'manager/max_vel': max_vel},               # 规划器允许的最大速度（m/s）
            {'manager/max_acc': max_acc},               # 规划器允许的最大加速度（m/s^2）
            {'manager/max_jerk': 4.0},                  # 允许的最大 jerk（加加速度），越大轨迹越“硬”
            {'manager/control_points_distance': 0.4},   # 相邻控制点之间的期望间距（m），影响轨迹细腻程度
            {'manager/feasibility_tolerance': 0.05},    # 可行性检查容差，略微放宽速度/加速度约束
            {'manager/path_ahead_time': path_ahead_time},  # 规划向前看的时间范围（s），影响局部目标位置
            {'manager/try_more_paths_and_choose_best': try_more_paths_and_choose_best},  # 是否尝试多条初始路径并选择代价最小的一条
            {'manager/drone_id': drone_id},             # 无人机/机器人编号，用于区分多机规划

            # ========== 轨迹优化代价函数权重（决定“更平滑”还是“更贴墙”等取舍） ==========
            {'optimization/lambda_smooth': 1.0},        # 平滑项权重：越大轨迹越圆滑
            {'optimization/lambda_collision': 0.5},     # 避障项权重：越大越“怕撞墙”，离障碍越远
            {'optimization/lambda_feasibility': 0.1},   # 速度/加速度可行性项权重：越大越不容易超限
            {'optimization/lambda_fitness': 5.0},       # 综合“贴合初始轨迹/参考路径”的约束权重，过小可能导致 refined 轨迹撞障
            {'optimization/dist0': 0.5},                # 避障距离阈值（m），小于该距离开始急剧增加代价
            {'optimization/swarm_clearance': 0.5},      # 群体间最小间距（m），用于多机避碰
            {'optimization/max_vel': max_vel},          # 优化内部使用的最大速度限制（与 manager/max_vel 对齐）
            {'optimization/max_acc': max_acc},          # 优化内部使用的最大加速度限制（与 manager/max_acc 对齐）

            # B-Spline parameters
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # Object prediction parameters
            {'prediction/obj_num': num_of_dynamic_objects},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    #=======================================================================================
    # Create LaunchDescription（启动计划本）
    launch_plan = LaunchDescription()

    launch_plan.add_action(map_size_x_arg)
    launch_plan.add_action(map_size_y_arg)
    launch_plan.add_action(map_size_z_arg)
    launch_plan.add_action(odometry_topic_arg)
    launch_plan.add_action(camera_pose_topic_arg)
    launch_plan.add_action(depth_topic_arg)
    launch_plan.add_action(cloud_topic_arg)
    launch_plan.add_action(frame_id_arg)
    launch_plan.add_action(cx_arg)
    launch_plan.add_action(cy_arg)
    launch_plan.add_action(fx_arg)
    launch_plan.add_action(fy_arg)
    launch_plan.add_action(max_vel_arg)
    launch_plan.add_action(max_acc_arg)
    launch_plan.add_action(path_ahead_time_arg)
    
    launch_plan.add_action(point_num_arg)
    launch_plan.add_action(point0_x_arg)
    launch_plan.add_action(point0_y_arg)
    launch_plan.add_action(point0_z_arg)
    launch_plan.add_action(point1_x_arg)
    launch_plan.add_action(point1_y_arg)
    launch_plan.add_action(point1_z_arg)
    launch_plan.add_action(point2_x_arg)
    launch_plan.add_action(point2_y_arg)
    launch_plan.add_action(point2_z_arg)
    launch_plan.add_action(point3_x_arg)
    launch_plan.add_action(point3_y_arg)
    launch_plan.add_action(point3_z_arg)
    launch_plan.add_action(point4_x_arg)
    launch_plan.add_action(point4_y_arg)
    launch_plan.add_action(point4_z_arg)
    
    launch_plan.add_action(egoplanner_input_point_or_path_arg)
    launch_plan.add_action(try_more_paths_and_choose_best_arg)
    launch_plan.add_action(plan_xy_only_arg)
    launch_plan.add_action(pct_path_skip_if_same_arg)
    launch_plan.add_action(num_of_dynamic_objects_arg)
    launch_plan.add_action(drone_id_arg)
    launch_plan.add_action(input_pose_message_type_arg)

    # Add Node
    launch_plan.add_action(ego_planner_node)

    return launch_plan
