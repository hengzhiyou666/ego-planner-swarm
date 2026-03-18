"""
真实机器狗启动：仅启动 ego_planner（规划器），订阅机器狗传感器/里程计话题。
用法见 docs/本地地图与机器狗相机配置说明.md
可通过 rviz:=false 关闭自动启动 RViz。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # =================先记住“我要用哪个参数”，等真正启动时再把实际值塞进来==================
    # ----- 通用参数（保留 drone_id 以复用原有命名空间/话题约定） -----
    drone_id = LaunchConfiguration('drone_id', default=0)
    obj_num = LaunchConfiguration('obj_num', default=10)
    map_size_x = LaunchConfiguration('map_size_x', default=90.0)  # 原为50.0
    map_size_y = LaunchConfiguration('map_size_y', default=130.0)  # 原为25.0
    map_size_z = LaunchConfiguration('map_size_z', default=10.0)  # 原为2.0

    # 调试模式：沿全局路径仅向前走指定米数；-1.0 表示关闭调试，正常使用完整 /pct_path
    # 使用浮点数以匹配节点中参数类型（double）
    debugMode_testGoForward_m = LaunchConfiguration('debugMode_testGoForward_m', default='-1.0')

    # ----- 真实机器狗开关（本工程清理后仅支持 True；保留参数避免用户脚本报错） -----
    use_real_robot = LaunchConfiguration('use_real_robot', default=True)

    # ----- 机器狗话题与相机参数（use_real_robot 时使用） -----
    odometry_topic = LaunchConfiguration('odometry_topic', default='/odometry')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/lidar_points')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/camera_pose')
    # 同时输入 depth + pose + cloud：depth 与 pose 做时间同步，cloud 独立订阅
    input_pose_message_type = LaunchConfiguration('input_pose_message_type', default=1)
    # 默认与 RViz 配置保持一致，使用 head_init 作为全局 / 地图坐标系
    frame_id = LaunchConfiguration('frame_id', default='head_init')
    # 是否实时显示占据栅格地图（1=显示，0=不显示；内部作为 bool 使用）
    enable_realtime_occupancy_grid = LaunchConfiguration('enable_realtime_occupancy_grid', default='1')
    # 是否自动启动 RViz（Fixed Frame: head_init；话题：/pct_path_unfinished, /odometry, /drone_0_plan_vis/optimal_list, /drone_0_plan_vis/goal_point）
    rviz = LaunchConfiguration('rviz', default='true')
    # pct_path 判重：True=与上次路径相同时跳过计算；False=不检测，每次都执行
    pct_path_skip_if_same = LaunchConfiguration('pct_path_skip_if_same', default=True)
    cx = LaunchConfiguration('cx', default='959.196655')
    cy = LaunchConfiguration('cy', default='538.812378')
    fx = LaunchConfiguration('fx', default='805.299072')
    fy = LaunchConfiguration('fy', default='805.879883')

    #=============新建一个“启动计划对象”，后面通过 add_action 添加各种动作（参数、节点、条件等）=======================
    # ----- 声明 Launch 参数 -----
    #声明启动参数 = 提前告诉系统：“这里有一个可以在启动时由用户/上层来决定的设置”，这样以后就能灵活调整，而不用每次都去改代码。
    launch_plan = LaunchDescription()#新建一个“启动计划对象”，后面通过 add_action 添加各种动作（参数、节点、条件等）
    launch_plan.add_action(DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID'))#加一个动作，声明一个启动参数
    launch_plan.add_action(DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size X (m)'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size Y (m)'))
    launch_plan.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size Z (m)'))
    launch_plan.add_action(DeclareLaunchArgument(
        'debugMode_testGoForward_m',
        default_value=debugMode_testGoForward_m,
        description='调试模式：>0 仅使用从起点向前指定米数的 /pct_path；-1.0 关闭调试，使用完整全局路径（类型为 double）'
    ))
    launch_plan.add_action(DeclareLaunchArgument('use_real_robot', default_value=use_real_robot,
                                        description='True: no map gen, no simulator; use robot topics'))
    launch_plan.add_action(DeclareLaunchArgument('odometry_topic', default_value=odometry_topic,
                                        description='Robot odometry topic'))
    launch_plan.add_action(DeclareLaunchArgument('depth_topic', default_value=depth_topic,
                                        description='Depth image topic (e.g. stereo depth)'))
    launch_plan.add_action(DeclareLaunchArgument('cloud_topic', default_value=cloud_topic,
                                        description='Point cloud topic'))
    launch_plan.add_action(DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic,
                                        description='Camera pose topic (used when input_pose_message_type=1)'))
    launch_plan.add_action(DeclareLaunchArgument('input_pose_message_type', default_value=input_pose_message_type,
                                        description='grid_map 输入位姿话题消息类型: 1=PoseStamped, 2=Odometry'))
    launch_plan.add_action(DeclareLaunchArgument('frame_id', default_value=frame_id,
                                        description='Planning/map frame id (odom/map)'))
    launch_plan.add_action(DeclareLaunchArgument(
        'enable_realtime_occupancy_grid',
        default_value=enable_realtime_occupancy_grid,
        description='Whether to publish occupancy grid pointcloud in realtime (1=on,0=off)'))
    launch_plan.add_action(DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx'))
    launch_plan.add_action(DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy'))
    launch_plan.add_action(DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx'))
    launch_plan.add_action(DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy'))
    launch_plan.add_action(DeclareLaunchArgument('rviz', default_value='true',
                                        description='Whether to auto-start RViz (config: Fixed Frame head_init, topics pct_path_unfinished, odometry, plan_vis)'))
    launch_plan.add_action(DeclareLaunchArgument('pct_path_skip_if_same', default_value='false',
                                        description='True: skip when /pct_path same as last; False: no check, always execute'))

    #=======================高级设置说明书advanced_param.launch.py===============================
    # ----- 规划器参数：真实机器狗用机器人话题与相机内参 -----
    advanced_param_for_real_robot = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource 是一个“帮你加载别的 .launch.py 文件”的小助手
        PythonLaunchDescriptionSource(str(get_package_share_directory('ego_planner')) + '/launch/advanced_param.launch.py'),#加载.launch.py
        launch_arguments={
            #给这本“高级说明书”传的参数列表（下面是规划器相关参数）
            # “参数名”: “参数值”，这里传的参数值是“我要用哪个参数”，等真正启动时再把实际值塞进来  
            
            #可从外部传入的参数：
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odometry_topic,
            'num_of_dynamic_objects': obj_num,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'cx': cx, 'cy': cy, 'fx': fx, 'fy': fy,
            'input_pose_message_type': input_pose_message_type,
            'frame_id': frame_id,
            'pct_path_skip_if_same': pct_path_skip_if_same,
            'debugMode_testGoForward_m': debugMode_testGoForward_m,
            'enable_realtime_occupancy_grid': enable_realtime_occupancy_grid,

            #规划器参数：不可从外部输入的，该处写好后固定的参数
            'max_vel': '2.0',  # 规划器允许的最大速度（单位：m/s）
            'max_acc': '1.0',  # 规划器允许的最大加速度（单位：m/s^2）
            'path_ahead_time': '7.5',  # 规划时间范围，向前看多长时间（单位：秒）
            'try_more_paths_and_choose_best': 'True',  # 是否“多算几条不同路径再从中挑一条最优路径”
            'egoplanner_input_point_or_path': '3',  # EGO Planner 输入是“单点 / 预设点 / 参考路径”等模式开关
            'plan_xy_only': 'False',#是否只规划XY平面，不规划Z轴（True：只规划XY平面2维路径，False：规划XYZ 3维路径）
            'point_num': '4',
            'point0_x': '31.2', 'point0_y': '-6.4', 'point0_z': '1.9',
            'point1_x': '33.0', 'point1_y': '-2.5', 'point1_z': '1.9',
            'point2_x': '36.8', 'point2_y': '20.3', 'point2_z': '1.5',
            'point3_x': '39.8', 'point3_y': '41.5', 'point3_z': '1.01',
            'point4_x': '44.4', 'point4_y': '53.2', 'point4_z': '0.9',
        }.items(),#把字典变成键值对列表
        condition=IfCondition(use_real_robot)#只有当 use_real_robot 为真（True）的时候，才执行这个 IncludeLaunchDescription；否则就跳过，不加载 advanced_param 那套。
    
    )
    launch_plan.add_action(advanced_param_for_real_robot)

    # ----- 可选：自动启动 RViz（Fixed Frame: head_init；话题见 config/robot.rviz） -----
    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'config', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz),
    )
    launch_plan.add_action(rviz_node)

    return launch_plan
