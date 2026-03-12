"""
真实机器狗启动：仅启动 ego_planner（规划器），订阅机器狗传感器/里程计话题。
用法见 docs/本地地图与机器狗相机配置说明.md
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # ----- 通用参数（保留 drone_id 以复用原有命名空间/话题约定） -----
    drone_id = LaunchConfiguration('drone_id', default=0)
    obj_num = LaunchConfiguration('obj_num', default=10)
    map_size_x = LaunchConfiguration('map_size_x', default=90.0)  # 原为50.0
    map_size_y = LaunchConfiguration('map_size_y', default=130.0)  # 原为25.0
    map_size_z = LaunchConfiguration('map_size_z', default=10.0)  # 原为2.0

    # ----- 真实机器狗开关（本工程清理后仅支持 True；保留参数避免用户脚本报错） -----
    use_real_robot = LaunchConfiguration('use_real_robot', default=True)

    # ----- 机器狗话题与相机参数（use_real_robot 时使用） -----
    odometry_topic = LaunchConfiguration('odometry_topic', default='/odometry')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/lidar_points')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/camera_pose')
    # 同时输入 depth + pose + cloud：depth 与 pose 做时间同步，cloud 独立订阅
    pose_type = LaunchConfiguration('pose_type', default=1)
    frame_id = LaunchConfiguration('frame_id', default='odom')
    cx = LaunchConfiguration('cx', default='959.196655')
    cy = LaunchConfiguration('cy', default='538.812378')
    fx = LaunchConfiguration('fx', default='805.299072')
    fy = LaunchConfiguration('fy', default='805.879883')

    # ----- 声明 Launch 参数 -----
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID'))
    ld.add_action(DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects'))
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size X (m)'))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size Y (m)'))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size Z (m)'))
    ld.add_action(DeclareLaunchArgument('use_real_robot', default_value=use_real_robot,
                                        description='True: no map gen, no simulator; use robot topics'))
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
    ld.add_action(DeclareLaunchArgument('frame_id', default_value=frame_id,
                                        description='Planning/map frame id (odom/map)'))
    ld.add_action(DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx'))
    ld.add_action(DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy'))
    ld.add_action(DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx'))
    ld.add_action(DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy'))
    # ----- 规划器参数：真实机器狗用机器人话题与相机内参 -----
    advanced_param_include_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_directory('ego_planner')) + '/launch/advanced_param.launch.py'),
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
            'frame_id': frame_id,
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
    ld.add_action(advanced_param_include_real)

    # ----- 可选：本地地图加载（发布 PCD 到 /map_generator/global_cloud） -----
    # 若需从 PCD 加载本地地图，可单独运行 map_loader 或发布 /map_generator/global_cloud；
    # 此处未内置 map_loader 节点，见 docs/本地地图与机器狗相机配置说明.md

    return ld
