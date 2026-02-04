# ego-planner-swarm

## 工程概况

ego-planner-swarm 是一个 **ROS 2（Humble）** 的无人机路径规划与仿真工程，包含：

- **规划相关**：ego_planner、plan_env、path_searching、bspline_opt、traj_utils、drone_detect 等
- **仿真相关**：uav_simulator 下多包（地图、控制、仿真、传感器等）

主入口包名为 **ego_planner**，launch 文件在 `src/planner/plan_manage/launch/`。

**快速上手**：按顺序完成 **[步骤一：编译](#步骤一编译)** → **[步骤二：终端 1 运行](#步骤二终端-1-运行启动-rviz)** → **[步骤三：终端 2 运行](#步骤三终端-2-运行启动规划与仿真)** 即可运行。

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

## 步骤二：终端 1 运行（启动 RViz）

> 打开**第一个终端**，执行以下命令启动 RViz 可视化。

```bash
source /opt/ros/humble/setup.bash
source /home/hzy/ego-planner-swarm/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

---

## 步骤三：终端 2 运行（启动规划与仿真）

> 打开**第二个终端**，先 source 再根据需求选择一种 launch 运行。

```bash
source /opt/ros/humble/setup.bash
source /home/hzy/ego-planner-swarm/install/setup.bash
```

- **单机：**
  ```bash
  ros2 launch ego_planner single_run_in_sim.launch.py
  ```
- **多机 swarm：**
  ```bash
  ros2 launch ego_planner swarm.launch.py
  ```
- **大规模 swarm：**
  ```bash
  ros2 launch ego_planner swarm_large.launch.py
  ```

可选参数示例（地图生成方式、是否考虑动力学）：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

## 小结

| 项目 | 说明 |
|------|------|
| 是否需编译 | 是，必须用 `colcon build` 编译工作空间 |
| 构建命令 | `colcon build`（在工程根目录） |
| 运行前 | `source install/setup.bash` |
| 启动方式 | 先 `rviz.launch.py`，再在另一终端用 `single_run_in_sim` / `swarm` / `swarm_large` 等 launch |

README 里没有写编译步骤，实际使用时要先完成上述编译和 `source install/setup.bash`，再按文档中的 `ros2 launch` 命令运行。
