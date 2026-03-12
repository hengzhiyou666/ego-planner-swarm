# ego-planner-swarm

## 工程概况

本仓库已裁剪为 **ROS 2（Humble）机器狗地面规划版 EGO Planner**，仅保留规划相关功能，包含：

- **规划相关**：ego_planner、plan_env、path_searching、bspline_opt、traj_utils、drone_detect 等
- **已删除**：无人机仿真/控制/传感器相关包（原 `uav_simulator/*`）、swarm/仿真 launch 与桥接等非机器狗必需模块

主入口包名为 **ego_planner**，launch 文件在 `src/planner/plan_manage/launch/`，推荐仅使用 `robot.launch.py`。

**快速上手**：完成 **[步骤一：编译](#步骤一编译)** 后，直接运行 **[步骤二：启动机器狗规划](#步骤二启动机器狗规划)**。

## 是否需要编译？

**需要。** 工程里有大量 C++ 源码（`.cpp`/`.hpp`），且是标准的 ROS 2 ament_cmake 工作空间，必须先编译再运行。

## 完整流程

### 环境与依赖（首次使用需配置）

- **ROS 2**：需要已安装 **ROS 2 Humble**（README 中用的是 `ros-humble-*`）。
- **系统库**：
  - **VTK**（安装 PCL 时依赖，编译 VTK 时需勾选 Qt）
  - **PCL**
- **DDS**：建议把默认 FastDDS 换成 CycloneDDS，否则可能很卡：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

检查是否生效：

```bash
ros2 doctor --report | grep -i "rmw"
# 应看到 rmw_cyclonedds_cpp
```

---

## 步骤一：编译

> 在工程根目录下执行以下命令完成编译。

```bash
cd /home/hzy/ego-planner-swarm
source /opt/ros/humble/setup.bash   # 若已写入 .bashrc 可省略
colcon build
```

或使用并行加速（可选）：

```bash
colcon build --parallel-workers 19 --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=19 --symlink-install
```

若只想编译 ego_planner 及其依赖：

```bash
colcon build --packages-up-to ego_planner
```

编译完成后加载工作空间（**每次新开终端运行前都要执行**，或写入 `~/.bashrc`）：

```bash
source install/setup.bash
```

---

## 步骤二：启动机器狗规划

> 打开终端，执行以下命令启动规划器（订阅机器狗里程计/深度/点云话题）。

```bash
source /opt/ros/humble/setup.bash
source /home/hzy/ego-planner-swarm/install/setup.bash
ros2 launch ego_planner robot.launch.py
```

常用参数（按你的机器狗话题实际填写）示例：

```bash
ros2 launch ego_planner robot.launch.py \
  use_real_robot:=True \
  odometry_topic:=/odometry \
  depth_topic:=/depth \
  cloud_topic:=/lidar_points \
  pose_type:=2
```

## 小结

| 项目 | 说明 |
|------|------|
| 是否需编译 | 是，必须用 `colcon build` 编译工作空间 |
| 构建命令 | `colcon build`（在工程根目录） |
| 运行前 | `source install/setup.bash` |
| 启动方式 | `ros2 launch ego_planner robot.launch.py` |

README 里没有写编译步骤，实际使用时要先完成上述编译和 `source install/setup.bash`，再按文档中的 `ros2 launch` 命令运行。
