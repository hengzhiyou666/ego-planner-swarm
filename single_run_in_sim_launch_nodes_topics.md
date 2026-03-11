# single_run_in_sim.launch.py 节点与话题说明

本文档描述 `ros2 launch ego_planner single_run_in_sim.launch.py` 启动的节点及其订阅/发布的话题。默认参数：`drone_id=0`，`use_mockamap=False`，`use_dynamic=False`。

---

## 一、启动的节点列表

| 序号 | 节点名 | 包/可执行文件 | 说明 |
|------|--------|----------------|------|
| 1 | `random_forest` | map_generator / random_forest | 随机森林地图生成（默认） |
| 2 | `drone_0_ego_planner_node` | ego_planner / ego_planner_node | 轨迹规划与 FSM |
| 3 | `drone_0_traj_server` | ego_planner / traj_server | B 样条→位置指令 |
| 4 | `drone_0_poscmd_2_odom` | poscmd_2_odom / poscmd_2_odom | 位置指令→里程计（仅 use_dynamic=False） |
| 5 | `drone_0_odom_visualization` | odom_visualization / odom_visualization | 里程计/路径可视化 |
| 6 | `drone_0_pcl_render_node` | local_sensing / pcl_render_node | 仿真深度/点云渲染 |

**说明：**

- `use_mockamap:=True` 时：会启动 `mockamap_node`，且**不**启动 `random_forest`。
- `use_dynamic:=True` 时：会额外启动 `drone_0_quadrotor_simulator_so3`、`drone_0_so3_control_container`，且**不**启动 `poscmd_2_odom`。

---

## 二、各节点输入/输出（订阅与发布）

### 1. random_forest（地图生成）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `odometry` | nav_msgs/Odometry | 当前位姿（launch 中无 remap） |
| **发布** | `/map_generator/local_cloud` | sensor_msgs/PointCloud2 | 局部点云 |
| **发布** | `/map_generator/global_cloud` | sensor_msgs/PointCloud2 | 全局点云 |
| **发布** | `/pcl_render_node/local_map` | sensor_msgs/PointCloud2 | 局部地图 |

---

### 2. drone_0_ego_planner_node（规划）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `drone_0_visual_slam/odom` | nav_msgs/Odometry | 里程计（remap 自 odom_world） |
| **订阅** | `drone_0_plan_vis/goal_point` | - | 目标点 |
| **订阅** | `drone_0_plan_vis/global_list`、`init_list`、`optimal_list`、`a_star_list` | - | 可视化列表 |
| **订阅** | `drone_0_pcl_render_node/cloud` | sensor_msgs/PointCloud2 | 点云（grid_map/cloud） |
| **订阅** | `drone_0_pcl_render_node/camera_pose` | geometry_msgs/PoseStamped | 相机位姿（grid_map/pose） |
| **订阅** | `drone_0_pcl_render_node/depth` | sensor_msgs/Image | 深度图（grid_map/depth） |
| **订阅** | `drone_0_grid/grid_map/occupancy_inflate` | sensor_msgs/PointCloud2 | 膨胀占据栅格 |
| **订阅** | `planning/broadcast_bspline_to_planner`（→ `/broadcast_bspline`） | traj_utils/Bspline | 广播 B 样条 |
| **订阅** | `/traj_start_trigger` | geometry_msgs/PoseStamped | PRESET 模式轨迹启动触发 |
| **订阅** | `/move_base_simple/goal` | geometry_msgs/PoseStamped | MANUAL 模式目标点 |
| **订阅** | `/drone_{id-1}_planning/swarm_trajs` | traj_utils/MultiBsplines | 多机时上一架机轨迹 |
| **发布** | `drone_0_planning/bspline` | traj_utils/Bspline | 规划 B 样条 |
| **发布** | `drone_0_planning/data_display` | traj_utils/DataDisp | 规划数据展示 |
| **发布** | `planning/broadcast_bspline_from_planner`（→ `/broadcast_bspline`） | traj_utils/Bspline | 广播 B 样条 |
| **发布** | `/drone_0_planning/swarm_trajs`（或 single 时 `/drone_single_planning/swarm_trajs`） | traj_utils/MultiBsplines | 多机时本机轨迹 |

---

### 3. drone_0_traj_server（轨迹伺服）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `drone_0_planning/bspline` | traj_utils/Bspline | 规划 B 样条（remap 自 planning/bspline） |
| **发布** | `drone_0_planning/pos_cmd` | quadrotor_msgs/PositionCommand | 位置指令（remap 自 position_cmd） |

---

### 4. drone_0_poscmd_2_odom（仅 use_dynamic=False）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `drone_0_planning/pos_cmd` | quadrotor_msgs/PositionCommand | 位置指令（remap 自 command） |
| **发布** | `drone_0_visual_slam/odom` | nav_msgs/Odometry | 仿真里程计（remap 自 odometry） |

---

### 5. drone_0_odom_visualization（可视化）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `drone_0_visual_slam/odom` | nav_msgs/Odometry | 里程计（remap 自 odom） |
| **发布** | `drone_0_vis/robot` | visualization_msgs/Marker | 机器人 mesh |
| **发布** | `drone_0_vis/path` | nav_msgs/Path | 路径 |
| **发布** | `drone_0_vis/time_gap` | std_msgs/Float64 | 时间间隔等 |

---

### 6. drone_0_pcl_render_node（仿真感知）

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **订阅** | `global_map`（→ `/map_generator/global_cloud`） | sensor_msgs/PointCloud2 | 全局点云 |
| **订阅** | `local_map` | sensor_msgs/PointCloud2 | 局部点云（若用） |
| **订阅** | `odometry`（→ `drone_0_visual_slam/odom`） | nav_msgs/Odometry | 里程计 |
| **发布** | `drone_0_pcl_render_node/cloud` | sensor_msgs/PointCloud2 | 渲染点云 |
| **发布** | `drone_0_pcl_render_node/depth` | sensor_msgs/Image | 深度图 |
| **发布** | `camera_pose`（launch 中规划器用 `pcl_render_node/camera_pose`） | geometry_msgs/PoseStamped | 相机位姿 |

---

## 三、主要话题数据流（简化）

```
random_forest
  订阅: odometry
  发布: /map_generator/global_cloud, /map_generator/local_cloud
        ↓
pcl_render_node  订阅: global_map(=global_cloud), odometry
  发布: drone_0_pcl_render_node/cloud, depth, camera_pose
        ↓
ego_planner_node 订阅: odom, cloud, depth, pose, occupancy_inflate, trigger/goal
  发布: drone_0_planning/bspline, data_display
        ↓
traj_server      订阅: drone_0_planning/bspline
  发布: drone_0_planning/pos_cmd
        ↓
poscmd_2_odom    订阅: drone_0_planning/pos_cmd
  发布: drone_0_visual_slam/odom  ──→ 闭环回 ego_planner、pcl_render、odom_visualization、random_forest(若接 odometry)
```

---

## 四、使用 mockamap 时（use_mockamap:=True）

- **mockamap_node**
  - **发布**：`/mock_map`（remap 到 `/map_generator/global_cloud`），类型 `sensor_msgs/PointCloud2`
  - **订阅**：无（自生成地图）

其余节点与上面一致，仅地图来源由 `random_forest` 变为 `mockamap`。

---

## 五、常用启动示例

```bash
# 默认仿真（单机、随机森林地图、理想动力学）
ros2 launch ego_planner single_run_in_sim.launch.py

# 使用 mockamap 地图，关闭动态仿真
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

修改 `drone_id` 或 `use_dynamic` 时，将上述话题中的 `drone_0` 替换为对应 `drone_{id}` 即可。
