#include <ego_planner/ego_replan_fsm.h>
#include <cmath>
#include <limits>

namespace ego_planner
{

  /**
   * 状态机初始化：绑定 node、加载 FSM 参数与预设路点、创建规划器与可视化、注册定时器与话题订阅/发布。
   * - 初始化状态（current_state_=STATE_ONE__WAIT_FOR_ODOM，have_target_/have_odom_/have_trigger_ 等）。
   * - 根据 target_type_ 订阅目标来源（MANUAL_TARGET→/move_base_simple/goal，PRESET_TARGET→/path_start_trigger 并阻塞至收到触发后 readGivenWps，USE_GLOBAL_PATH→/pct_path）。
   * - 创建 10ms 执行定时器与 100ms 安全检测定时器，以及 odom、swarm_paths、bspline 等订阅/发布。
   */
  void classEGOPlannerStateMachine::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    have_pct_path_ = false;
    current_state_ = FSM_EXEC_STATE::STATE_ONE__WAIT_FOR_ODOM;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/egoplanner_input_point_or_path", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/path_ahead_time", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/plan_xy_only", false);
    node_->declare_parameter("fsm/pct_path_skip_if_same", false);  // 默认不判重，与 robot.launch 一致
    node_->declare_parameter("fsm/debugMode_testGoForward_m", 10.0); // Debug 前进测试模式：从 /pct_path 起点向前截取多少米，-1 表示关闭
    node_->get_parameter("fsm/egoplanner_input_point_or_path", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/path_ahead_time", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/plan_xy_only", plan_xy_only_);
    node_->get_parameter("fsm/debugMode_testGoForward_m", debugMode_testGoForward_m_);
    // launch 传入的布尔常为字符串 "True"/"False"，需兼容解析
    {
      auto p = node_->get_parameter("fsm/pct_path_skip_if_same");
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        pct_path_skip_if_same_ = p.as_bool();
      else if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        const std::string s = p.as_string();
        pct_path_skip_if_same_ = (s == "true" || s == "True" || s == "1");
      }
      // 否则保持成员默认值 true
      RCLCPP_INFO(node_->get_logger(), "fsm/pct_path_skip_if_same = %s (判重%s)",
                  pct_path_skip_if_same_ ? "true" : "false",
                  pct_path_skip_if_same_ ? "开启" : "关闭");
    }

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_index_now_);

    for (int i = 0; i < waypoint_index_now_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      // 从参数服务器读取第 i 个预设路点的 x/y/z，写入 waypoints_array_
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_array_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_array_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_array_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverPathToOptimizer(); // store trajectories
    planner_manager_->setDogIdtoOpt();

    //============================================回调函数=========================================================
    /* callback */
    // 执行定时器：每 10 ms 调用一次 FSM 回调，驱动状态机执行与轨迹跟踪
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&classEGOPlannerStateMachine::function_runWhichStateNow_every10ms, this));

    // 安全定时器：每 100 ms 调用一次碰撞检测回调，检查障碍物并触发重规划
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&classEGOPlannerStateMachine::autofunction_checkStoneCallback_every100ms, this));

    //============================================订阅话题=========================================================
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->autoFunction_GetOdometry(msg);
        });
    // std::bind(&classEGOPlannerStateMachine::autoFunction_GetOdometry, this, std::placeholders::_1));

    //============================================发布话题=========================================================
    if (planner_manager_->pp_.dog_id >= 1)
    {
      string sub_topic_name = string("/dog_") + std::to_string(planner_manager_->pp_.dog_id - 1) + string("_planning/swarm_paths");
      swarm_paths_sub_ = node_->create_subscription<path_tools::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const path_tools::msg::MultiBsplines> &msg)
          {
            this->swarmPathsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/dog_") + std::to_string(planner_manager_->pp_.dog_id) + string("_planning/swarm_paths");
    string pub_topic_name;
    if (planner_manager_->pp_.dog_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single dog:%d", planner_manager_->pp_.dog_id);
      pub_topic_name = string("/dog_") + "single" + string("_planning/swarm_paths");
    }else
    {
      pub_topic_name = string("/dog_") + std::to_string(planner_manager_->pp_.dog_id) + string("_planning/swarm_paths");
    }
    
    swarm_paths_pub_ = node_->create_publisher<path_tools::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<path_tools::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<path_tools::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const path_tools::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<path_tools::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<path_tools::msg::DataDisp>("planning/data_display", 100);
    // 发布“从当前里程计位置开始、尚未走完的参考路径”，方便在 RViz 中查看参考路径还剩下多少
    pct_path_unfinished_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/pct_path_unfinished", 1);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/path_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }

    //==============================选取全局路径中的靠近自身的一小段作为局部路径规划的目标路径============================================
    else if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH)
    {
      pct_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "/pct_path",
          1,
          [this](const std::shared_ptr<const nav_msgs::msg::Path> &msg)
          {
            this->pctPathCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "USE_GLOBAL_PATH mode: waiting for /pct_path and odom.");
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  /**
   * 根据当前 waypoints_array_ / waypoint_index_now_ 生成内部路点序列 waypoints_array_xyz_，并触发全局规划。
   * - 将 waypoints_array_ 拷贝到 waypoints_array_xyz_，wp_id_ 置 0。
   * - USE_GLOBAL_PATH 且多路点：在路径上找离 odom 最近点，向前约 2m 得首目标，用该子路径做 waypoints_array_xyz_，
   *   然后 planGlobalPathWaypoints；成功则置 have_target_、have_new_target_ 并切到 GEN_NEW_PATH/REPLAN_PATH，
   *   失败则退化为只规划到第一个路点。
   * - 其他情况（单路点、PRESET_TARGET、或上述失败）：仅对路点做可视化，并调用 planNextWaypoint(waypoints_array_xyz_[0]) 规划到第一个路点。
   */
  void classEGOPlannerStateMachine::readGivenWps()
  {
    if (waypoint_index_now_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_index_now_ = %d", waypoint_index_now_);
      return;
    }

    waypoints_array_xyz_.resize(waypoint_index_now_);
    for (int i = 0; i < waypoint_index_now_; i++)
    {
      waypoints_array_xyz_[i](0) = waypoints_array_[i][0];
      waypoints_array_xyz_[i](1) = waypoints_array_[i][1];
      waypoints_array_xyz_[i](2) = plan_xy_only_ ? 0.0 : waypoints_array_[i][2];
    }

    wp_id_ = 0;

    // ========== 分支：USE_GLOBAL_PATH 且多路点（>1）时，从路径上找“最近点 + 向前 2m”作为首目标，再整段多路点全局规划 ==========
    if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH && waypoint_index_now_ > 1)
    {
      // ---------- 工具：求点 p 到线段 ab 的最近点及参数 t ----------
      auto closestOnSegment = [](const Eigen::Vector3d &p, const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                                 Eigen::Vector3d &out_closest, double &out_t) -> double {
        Eigen::Vector3d ap = p - a, ab = b - a;
        double ab2 = ab.squaredNorm();
        if (ab2 < 1e-12)
        {
          out_closest = a;
          out_t = 0.0;
          return (p - a).squaredNorm();
        }
        double t = (ap.dot(ab) / ab2);
        t = std::max(0.0, std::min(1.0, t));
        out_t = t;
        out_closest = a + t * ab;
        return (p - out_closest).squaredNorm();
      };

      // ---------- 模块 1：找路径上离当前位置最近的点 join_pt（所在线段索引 seg_idx） ----------
      Eigen::Vector3d join_pt = waypoints_array_xyz_[0];
      double join_t = 0.0;
      double best_d2 = 1e30;
      int seg_idx = 0;
      Eigen::Vector3d cand;
      double cand_t = 0.0;
      for (int i = 0; i < waypoint_index_now_ - 1; i++)
      {
        double d2 = closestOnSegment(robot_location_now_fromOdomDirectly_, waypoints_array_xyz_[i], waypoints_array_xyz_[i + 1], cand, cand_t);
        if (d2 < best_d2)
        {
          best_d2 = d2;
          join_pt = cand;
          join_t = cand_t;
          seg_idx = i;
        }
      }
      if (plan_xy_only_)
        join_pt(2) = 0.0;

      // ---------- 模块 2：从 join_pt 沿路径方向向前约 2m，得到首目标 first_pt ----------
      const double forward_dist = 2.0;
      double remain = forward_dist;
      Eigen::Vector3d first_pt = join_pt;

      // 从当前线段的 join_t 位置开始往前推
      Eigen::Vector3d a = waypoints_array_xyz_[seg_idx];
      Eigen::Vector3d b = waypoints_array_xyz_[seg_idx + 1];
      if (plan_xy_only_)
      {
        a(2) = 0.0;
        b(2) = 0.0;
      }
      Eigen::Vector3d ab = b - a;
      double seg_len = ab.norm();
      double dist_to_b = (1.0 - join_t) * seg_len;

      if (seg_len < 1e-6)
      {
        first_pt = join_pt;
      }
      else if (remain <= dist_to_b)
      {
        // 2m 落在同一段内
        double t2 = join_t + remain / seg_len;
        first_pt = a + t2 * ab;
      }
      else
      {
        // 先走到 b，再继续沿后续路段累计
        remain -= dist_to_b;
        first_pt = b;
        int j = seg_idx + 1;
        while (remain > 1e-6 && j < waypoint_index_now_ - 1)
        {
          Eigen::Vector3d p0 = waypoints_array_xyz_[j];
          Eigen::Vector3d p1 = waypoints_array_xyz_[j + 1];
          if (plan_xy_only_)
          {
            p0(2) = 0.0;
            p1(2) = 0.0;
          }
          Eigen::Vector3d d = p1 - p0;
          double L = d.norm();
          if (L < 1e-6)
          {
            j++;
            continue;
          }
          if (remain <= L)
          {
            first_pt = p0 + (remain / L) * d;
            remain = 0.0;
            break;
          }
          remain -= L;
          first_pt = p1;
          j++;
        }
      }
      if (plan_xy_only_)
        first_pt(2) = 0.0;

      // ---------- 模块 3：用 first_pt 作为首点，拼接其后路径到终点，更新 waypoints_array_xyz_ 与 waypoint_index_now_ ----------
      std::vector<Eigen::Vector3d> new_wps;
      new_wps.reserve(waypoint_index_now_ + 1);
      new_wps.push_back(first_pt);
      for (int i = seg_idx + 1; i < waypoint_index_now_; i++)
      {
        if ((waypoints_array_xyz_[i] - first_pt).norm() > 1e-3)
          new_wps.push_back(waypoints_array_xyz_[i]);
      }
      waypoints_array_xyz_ = new_wps;
      waypoints_array_xyz_ = waypoints_array_xyz_;//heng20260315
      waypoint_index_now_ = (int)waypoints_array_xyz_.size();
      wp_id_ = 0;

      // ---------- 模块 4：对更新后的 waypoints_array_xyz_ 做路点可视化 ----------
      for (size_t i = 0; i < (size_t)waypoint_index_now_; i++)
      {
        visualization_->displayGoalPoint(waypoints_array_xyz_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // ---------- 模块 5：构造 waypoints_vec 并调用多路点全局规划（含沿路径方向的速度约束） ----------
      std::vector<Eigen::Vector3d> waypoints_vec;
      for (int i = 0; i < waypoint_index_now_; i++)
        waypoints_vec.push_back(waypoints_array_xyz_[i]);

      bool success = planner_manager_->planGlobalPathWaypoints(
          robot_location_now_fromOdomDirectly_,
          [&]() -> Eigen::Vector3d {
            // 用“沿路径前进方向”的速度约束生成全局参考轨迹，避免因 robot_vel_now_fromOdomDirectly_ 横向/反向导致 min-snap 轨迹折返
            Eigen::Vector3d v = robot_vel_now_fromOdomDirectly_;
            if (plan_xy_only_)
              v(2) = 0.0;
            Eigen::Vector3d dir = first_pt - robot_location_now_fromOdomDirectly_;
            if (plan_xy_only_)
              dir(2) = 0.0;
            if (dir.norm() < 1e-3)
              return Eigen::Vector3d::Zero();
            Eigen::Vector3d u = dir.normalized();
            double s = v.dot(u);
            if (s <= 0.0)
              return Eigen::Vector3d::Zero();
            return s * u; // 只保留沿前进方向的速度分量
          }(),
          Eigen::Vector3d::Zero(),
          waypoints_vec, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

      // ---------- 模块 6：规划成功则置 end_pt_/have_target_、可视化全局路径并切换状态后 return ----------
      if (success)
      {
        end_pt_ = waypoints_array_xyz_[wp_id_];
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_path_afterCalculate_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> global_path(i_end);
        for (int i = 0; i < i_end; i++)
        {
          global_path[i] = planner_manager_->global_path_afterCalculate_.global_path_.evaluate(i * step_size_t);
          if (plan_xy_only_)
            global_path[i](2) = 0.0;
        }
        visualization_->displayGlobalPathList(global_path, 0.1, 0);

        // 首次收到全局路径且当前还在“等待里程计”或“等待目标”阶段时，应该先进入 GEN_NEW_PATH，
        // 生成第一条局部轨迹，再由 EXEC_PATH 驱动后续 REPLAN_PATH；
        // 否则可能在还没有任何 local_data_ 轨迹的情况下直接进入 REPLAN_PATH，导致 planFromCurrentPath 访问未初始化数据而段错误。
        if (current_state_ == WAIT_TARGET || current_state_ == STATE_ONE__WAIT_FOR_ODOM)
          changeStateTo(GEN_NEW_PATH, "TRIG");
        else
          changeStateTo(REPLAN_PATH, "TRIG");
        return;
      }
      // 全路径规划失败时退化为只规划到第一个路点（落到下方“单路点/非 USE_GLOBAL_PATH”逻辑）
    }

    // ---------- 模块 7：非 USE_GLOBAL_PATH 或单路点——仅做路点可视化 ----------
    for (size_t i = 0; i < (size_t)waypoint_index_now_; i++)
    {
      visualization_->displayGoalPoint(waypoints_array_xyz_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // ---------- 模块 8：规划到当前第一个路点 waypoints_array_xyz_[wp_id_] ----------
    planNextWaypoint(waypoints_array_xyz_[wp_id_]);
  }

  bool classEGOPlannerStateMachine::buildUnfinishedFromGlobalPath(const std::vector<Eigen::Vector3d> &globalpath_points,
                                                             const Eigen::Vector3d &robot_location,
                                                             std::vector<Eigen::Vector3d> &unfinished_points_out,
                                                             nav_msgs::msg::Path *unfinished_path_out,
                                                             const std_msgs::msg::Header *path_header)
  {
    if (globalpath_points.size() < 2)
      return false;
    unfinished_points_out.clear();

    // 在全局路径中找离 robot_location 最近的点 b
    int b_index = 0;
    double minDistance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(globalpath_points.size()); ++i)
    {
      double d2 = (globalpath_points[i] - robot_location).squaredNorm();
      if (d2 < minDistance)
      {
        minDistance = d2;
        b_index = i;
      }
    }
    Eigen::Vector3d b = globalpath_points[b_index];
    double ab = (b - robot_location).norm();

    // 从 b 沿路径向前累积弧长 ab，得到点 c
    Eigen::Vector3d cPointXyz = b;
    int c_index = b_index;
    if (ab > 0.1 && b_index < static_cast<int>(globalpath_points.size()) - 1)
    {
      double abRemain = ab;
      Eigen::Vector3d currentPoint = b;
      bool find_c = false;
      for (int i = b_index; i < static_cast<int>(globalpath_points.size()) - 1; ++i)
      {
        Eigen::Vector3d nextPoint = globalpath_points[i + 1];
        double seg = (nextPoint - currentPoint).norm();
        if (seg < 1e-3)
        {
          currentPoint = nextPoint;
          continue;
        }
        if (abRemain <= seg)
        {
          double ratio = abRemain / seg;
          cPointXyz = currentPoint + ratio * (nextPoint - currentPoint);
          c_index = i;
          find_c = true;
          break;
        }
        abRemain -= seg;
        currentPoint = nextPoint;
      }
      if (!find_c)
      {
        cPointXyz = globalpath_points.back();
        c_index = static_cast<int>(globalpath_points.size()) - 2;
      }
    }

    const double oneStepLength = 0.1;
    unfinished_points_out.reserve(static_cast<int>(ab / 0.1) + globalpath_points.size() + 1);

    Eigen::Vector3d ac_vector = cPointXyz - robot_location;
    double ac_len = ac_vector.norm();
    unfinished_points_out.push_back(robot_location);
    if (ac_len > 0.2)
    {
      int stepsNumber = static_cast<int>(std::floor(ac_len / oneStepLength));
      Eigen::Vector3d ac_unit = ac_vector / ac_len;
      for (int i = 1; i < stepsNumber; i++)
        unfinished_points_out.push_back(robot_location + (i * oneStepLength) * ac_unit);
    }
    unfinished_points_out.push_back(cPointXyz);
    for (int i = c_index + 1; i < static_cast<int>(globalpath_points.size()); ++i)
      unfinished_points_out.push_back(globalpath_points[i]);

    if (unfinished_path_out && path_header)
    {
      unfinished_path_out->header = *path_header;
      unfinished_path_out->poses.resize(unfinished_points_out.size());
      for (size_t i = 0; i < unfinished_points_out.size(); ++i)
      {
        auto &ps = unfinished_path_out->poses[i];
        ps.header = *path_header;
        ps.pose.position.x = unfinished_points_out[i].x();
        ps.pose.position.y = unfinished_points_out[i].y();
        ps.pose.position.z = unfinished_points_out[i].z();
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;
      }
    }
    return true;
  }

  void classEGOPlannerStateMachine::pctPathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &globalpath)
  {
    cout << "检测到有新的/pct_path话题被发布，进入pctPathCallback()回调函数" << endl;
    // 如果此时还没有里程计，就没法知道“当前位置 a 在路径上的什么位置”，只能先忽略
    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(), "/pct_path received but odom not ready yet.");
      return;
    }

    // 1）先拷贝一份路径，方便在 Debug 模式下对其进行截断等操作
    nav_msgs::msg::Path current_path = *globalpath;

    // 1.1）安全性检查：如果消息里一个点都没有，直接忽略，不进行后续处理
    if (current_path.poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "we have got globalpath, but it is empty!!! so we ignore it,return now.");
      return;
    }
    if (pct_path_skip_if_same_)
    {
      cout << "进行全局路径重复检查，pct_path_skip_if_same_为True=================================================" << endl;
    }
    else
    {
      cout << "不进行全局路径重复检查，pct_path_skip_if_same_为False=================================================" << endl;
    }
    // 1.2）Debug 前进测试模式：从起点开始只保留前 debugMode_testGoForward_m_ 米
    if (debugMode_testGoForward_m_ > 0.0 && current_path.poses.size() >= 2)
    {
      const double deubgLength10m = debugMode_testGoForward_m_;
      nav_msgs::msg::Path truncated;
      truncated.header = current_path.header;
      truncated.poses.clear();
      truncated.poses.reserve(current_path.poses.size());

      truncated.poses.push_back(current_path.poses.front());
      double totalLength = 0.0;

      for (size_t i = 1; i < current_path.poses.size(); ++i)
      {
        const auto &prev = current_path.poses[i - 1].pose.position;
        const auto &curr = current_path.poses[i].pose.position;
        const double dx = curr.x - prev.x;
        const double dy = curr.y - prev.y;
        const double dz = curr.z - prev.z;
        const double twoPointTempLength = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (twoPointTempLength < 1e-6)
          continue;

        if (totalLength + twoPointTempLength <= deubgLength10m + 1e-3)
        {
          truncated.poses.push_back(current_path.poses[i]);
          totalLength += twoPointTempLength;
          if (totalLength >= deubgLength10m - 1e-3)
            break;
        }
        else
        {
          // 在当前段内插值得到恰好 deubgLength10m 处的点
          const double remain = deubgLength10m - totalLength;
          if (remain > 0.0)
          {
            const double ratio = remain / twoPointTempLength;
            geometry_msgs::msg::PoseStamped interp = current_path.poses[i - 1];
            interp.pose.position.x = prev.x + ratio * dx;
            interp.pose.position.y = prev.y + ratio * dy;
            interp.pose.position.z = prev.z + ratio * dz;
            truncated.poses.push_back(interp);
          }
          break;
        }
      }

      // 如果整个路径长度都小于 target_len，就保持原路径不变；否则用截断后的路径
      if (truncated.poses.size() >= 2)
      {
        current_path = std::move(truncated);
      }
    }

    // 1.5）路径判重（可由参数关闭）：与上一帧路径相同则直接退出，不重复计算
    if (pct_path_skip_if_same_)
    {
      constexpr double kPathCompareTol = 1e-6;
      if (!last_pct_path_.poses.empty() &&
          last_pct_path_.header.frame_id == current_path.header.frame_id &&
          last_pct_path_.poses.size() == current_path.poses.size())
      {
        bool same = true;
        for (size_t i = 0; i < current_path.poses.size(); ++i)
        {
          const auto &a = last_pct_path_.poses[i].pose.position;
          const auto &b = current_path.poses[i].pose.position;
          if (std::abs(a.x - b.x) > kPathCompareTol || std::abs(a.y - b.y) > kPathCompareTol || std::abs(a.z - b.z) > kPathCompareTol)
          {
            same = false;
            break;
          }
        }
        if (same)
        {
          RCLCPP_DEBUG(node_->get_logger(), "/pct_path 与上次相同，跳过计算。");
          return;
        }
      }
    }

    // 2）把（可能已经被 Debug 截断后的）/pct_path 转成 Eigen 点列，并得到“未走完路径” unfinished_points
    std::vector<Eigen::Vector3d> globalpath_points;
    globalpath_points.reserve(current_path.poses.size());
    for (const auto &ps : current_path.poses)
    {
      const auto &p = ps.pose.position;
      Eigen::Vector3d pt(p.x, p.y, p.z);
      if (plan_xy_only_)
        pt(2) = 0.0;
      globalpath_points.push_back(pt);
    }
    if (globalpath_points.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "全局路径点太少了，globalpath_points has less than 2 points, ignore.");
      return;
    }
    Eigen::Vector3d robot_location = robot_location_now_fromOdomDirectly_;
    if (plan_xy_only_)
      robot_location(2) = 0.0;

    std::vector<Eigen::Vector3d> unfinished_points;
    nav_msgs::msg::Path unfinished_path;
    if (!buildUnfinishedFromGlobalPath(globalpath_points, robot_location, unfinished_points, &unfinished_path, &current_path.header))
    {
      return;
    }
    if (unfinished_points.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "buildUnfinishedFromGlobalPath produced too few points.");
      return;
    }
    pct_path_unfinished_pub_->publish(unfinished_path);

    // 6）从 pct_path_unfinished（即 unfinished_points）中按顺序全取前 7m 的点，去掉最前5个点，作为局部规划引导段送给 EGO Planner
    const double guide_len = 7.0;  // 单位 m
    std::vector<Eigen::Vector3d> first_7m_pts;
    first_7m_pts.reserve(unfinished_points.size());
    double templength = 0.0;
    for (size_t i = 1; i < unfinished_points.size(); ++i)
    {
      first_7m_pts.push_back(unfinished_points[i]);
      if (i > 0)
      {
        templength += (unfinished_points[i] - unfinished_points[i - 1]).norm();
        if (templength >= guide_len - 1e-3)
          break;
      }
    }
    // 去掉最前面的 5 个点后送给 EGO Planner
    guide_path_7m_withoutFirst5points_fsm_h_.clear();
    if (first_7m_pts.size() > 5)
    {
      guide_path_7m_withoutFirst5points_fsm_h_.insert(guide_path_7m_withoutFirst5points_fsm_h_.end(), first_7m_pts.begin() + 5, first_7m_pts.end());
    }
    
    
    //=====================================路点约束=========================================================
    // 7）waypoints_array_ 仍用于全局多路点规划：取前 7m 填 waypoints_array_，供 readGivenWps 使用
    waypoint_index_now_ = 0;
    const size_t max_wp = 200;
    templength = 0.0;
    for (size_t i = 1; i < unfinished_points.size() && waypoint_index_now_ < static_cast<int>(max_wp); ++i)
    {
      const Eigen::Vector3d &p = unfinished_points[i];
      waypoints_array_[waypoint_index_now_][0] = p.x();
      waypoints_array_[waypoint_index_now_][1] = p.y();
      waypoints_array_[waypoint_index_now_][2] = plan_xy_only_ ? 0.0 : p.z();
      waypoint_index_now_++;
      if (i > 0)
      {
        templength += (unfinished_points[i] - unfinished_points[i - 1]).norm();
        if (templength >= guide_len - 1e-3)
          break;
      }
    }
    if (waypoint_index_now_ <= 0)
    {
      RCLCPP_WARN(node_->get_logger(), "No valid waypoints generated from /pct_path_unfinished.");
      return;
    }

    // 标记“已经有一条参考路径了”，并把它当作一次“开始规划”的触发信号
    have_pct_path_ = true;
    have_trigger_ = true; /* USE_GLOBAL_PATH 下用收到路径作为触发 */

    // 8）把采样好的 waypoints_array_ 转换成内部的 waypoints_array_xyz_ 向量，并调用原有多路点规划逻辑
    readGivenWps();

    // 保存当前路径（已考虑 Debug 截断），供下次回调判重
    last_pct_path_ = current_path;
  }

  void classEGOPlannerStateMachine::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    Eigen::Vector3d wp = next_wp;
    if (plan_xy_only_)
      wp(2) = 0.0;
    bool success = false;
    success = planner_manager_->planGlobalPath(robot_location_now_fromOdomDirectly_, robot_vel_now_fromOdomDirectly_, Eigen::Vector3d::Zero(), wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_path_afterCalculate_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> global_path(i_end);
      for (int i = 0; i < i_end; i++)
      {
        global_path[i] = planner_manager_->global_path_afterCalculate_.global_path_.evaluate(i * step_size_t);
        if (plan_xy_only_)
          global_path[i](2) = 0.0;
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM状态转换 ***/
      if (current_state_ == WAIT_TARGET)
        changeStateTo(GEN_NEW_PATH, "TRIG");
      else
      {
        /* 已在 executor 内，不再阻塞 spin_some，避免 "Node has already been added to an executor" */
        changeStateTo(REPLAN_PATH, "TRIG");
      }

      visualization_->displayGlobalPathList(global_path, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void classEGOPlannerStateMachine::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = robot_location_now_fromOdomDirectly_;
  }

  void classEGOPlannerStateMachine::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = robot_location_now_fromOdomDirectly_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void classEGOPlannerStateMachine::autoFunction_GetOdometry(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    robot_location_now_fromOdomDirectly_(0) = msg->pose.pose.position.x;
    robot_location_now_fromOdomDirectly_(1) = msg->pose.pose.position.y;
    robot_location_now_fromOdomDirectly_(2) = plan_xy_only_ ? 0.0 : msg->pose.pose.position.z;

    robot_vel_now_fromOdomDirectly_(0) = msg->twist.twist.linear.x;
    robot_vel_now_fromOdomDirectly_(1) = msg->twist.twist.linear.y;
    robot_vel_now_fromOdomDirectly_(2) = plan_xy_only_ ? 0.0 : msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void classEGOPlannerStateMachine::BroadcastBsplineCallback(const std::shared_ptr<const path_tools::msg::Bspline> &msg)
  {
    size_t id = msg->dog_id;
    if ((int)id == planner_manager_->pp_.dog_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->dog_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->dog_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_paths_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_paths_buf_.size(); i <= id; i++)
      {
        OnePathDataOfSwarm blank;
        blank.dog_id = -1;
        planner_manager_->swarm_paths_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - robot_location_now_fromOdomDirectly_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_paths_buf_[id].dog_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_paths_buf_[id].dog_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_paths_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_paths_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_path(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_path.setKnot(knots);
    planner_manager_->swarm_paths_buf_[id].position_path_ = pos_path;

    planner_manager_->swarm_paths_buf_[id].start_pos_ = planner_manager_->swarm_paths_buf_[id].position_path_.evaluateDeBoorT(0);

    planner_manager_->swarm_paths_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeStateTo(REPLAN_PATH, "TRAJ_CHECK");
    }
  }

  void classEGOPlannerStateMachine::swarmPathsCallback(const std::shared_ptr<const path_tools::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.path.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmPathsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->path.size() != msg->dog_id_from + 1) // dog_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->path.size()=%d, msg->dog_id_from+1=%d", (int)msg->path.size(), msg->dog_id_from + 1);
      return;
    }

    if (msg->path[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_paths_buf_.clear();
    planner_manager_->swarm_paths_buf_.resize(msg->path.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->path.size(); i++)
    {

      Eigen::Vector3d cp0(msg->path[i].pos_pts[0].x, msg->path[i].pos_pts[0].y, msg->path[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->path[i].pos_pts[1].x, msg->path[i].pos_pts[1].y, msg->path[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->path[i].pos_pts[2].x, msg->path[i].pos_pts[2].y, msg->path[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - robot_location_now_fromOdomDirectly_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_paths_buf_[i].dog_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->path[i].pos_pts.size());
      Eigen::VectorXd knots(msg->path[i].knots.size());
      for (size_t j = 0; j < msg->path[i].knots.size(); ++j)
      {
        knots(j) = msg->path[i].knots[j];
      }
      for (size_t j = 0; j < msg->path[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->path[i].pos_pts[j].x;
        pos_pts(1, j) = msg->path[i].pos_pts[j].y;
        pos_pts(2, j) = msg->path[i].pos_pts[j].z;
      }

      planner_manager_->swarm_paths_buf_[i].dog_id = i;

      // 计算路径持续时间
      if (msg->path[i].order % 2)
      {
        double cutback = (double)msg->path[i].order / 2 + 1.5;
        planner_manager_->swarm_paths_buf_[i].duration_ = msg->path[i].knots[msg->path[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->path[i].order / 2 + 1.5;
        planner_manager_->swarm_paths_buf_[i].duration_ = (msg->path[i].knots[msg->path[i].knots.size() - floor(cutback)] + msg->path[i].knots[msg->path[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_paths_buf_[i].position_path_ =
      UniformBspline pos_path(pos_pts, msg->path[i].order, msg->path[i].knots[1] - msg->path[i].knots[0]);
      pos_path.setKnot(knots);
      planner_manager_->swarm_paths_buf_[i].position_path_ = pos_path;

      planner_manager_->swarm_paths_buf_[i].start_pos_ = planner_manager_->swarm_paths_buf_[i].position_path_.evaluateDeBoorT(0);

      planner_manager_->swarm_paths_buf_[i].start_time_ = msg->path[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void classEGOPlannerStateMachine::changeStateTo(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == current_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"STATE_ONE__WAIT_FOR_ODOM", "WAIT_TARGET", "GEN_NEW_PATH", "REPLAN_PATH", "EXEC_PATH", "EMERGENCY_STOP"};
    int pre_s = int(current_state_);
    current_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, classEGOPlannerStateMachine::FSM_EXEC_STATE> classEGOPlannerStateMachine::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, current_state_);
  }

  void classEGOPlannerStateMachine::printCurrentState()
  {
    static string state_str[7] = {"STATE_ONE__WAIT_FOR_ODOM", "WAIT_TARGET", "GEN_NEW_PATH", "REPLAN_PATH", "EXEC_PATH", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(current_state_)] << ",当前状态是: " + state_str[int(current_state_)] << endl;
  }

  // 状态机主循环（由 10ms 定时器周期性调用）：根据当前 current_state_ 执行对应逻辑并驱动状态迁移（STATE_ONE__WAIT_FOR_ODOM→WAIT_TARGET→规划→EXEC_PATH/REPLAN_PATH 等）
  void classEGOPlannerStateMachine::function_runWhichStateNow_every10ms()
  {
    // ----- 防止本次回调还没跑完、下一次又来了，先停掉定时器，最后再 reset -----
    exec_timer_->cancel(); // To avoid blockage

    // ----- 每跑满 100 次就打印一次当前状态和“有没有 odom/目标”，方便看日志 -----
    static int already_run_times_10ms = 0;
    already_run_times_10ms++;
    if (already_run_times_10ms == 100)
    {
      printCurrentState();
      if (!have_odom_)
        cout << "no odom，无法知道当前位置" << endl;
      if (!have_target_)
        cout << "wait for goal or trigger，等待输入目的地" << endl;
      already_run_times_10ms = 0;
    }

    switch (current_state_)
    {
    //===============================1/6 state_one__wait_for_odom===============================================================
    // ----- 初始化：有里程计了就切到“等目标”，没有就啥也不干直接走人 -----
    case STATE_ONE__WAIT_FOR_ODOM:
    {
      cout << "[当前在状态机里]当前状态是：STATE_ONE__WAIT_FOR_ODOM" << endl;
      if (have_odom_)
      {
        cout<<"which中，从STATE_ONE__WAIT_FOR_ODOM状态切换到WAIT_TARGET状态"<<endl;
        changeStateTo(WAIT_TARGET, "FSM");
        break;
      }
      else
      {
        goto force_return;
      }
    }

    //===============================2/6 wait_target===============================================================
    // ----- 等目标：还没收到目标或触发信号就 return；都有了就进“生成新路径”去算第一条轨迹 -----
    case WAIT_TARGET:
    {
      cout << "[当前在状态机里]当前状态是：WAIT_TARGET" << endl;
      if (have_target_ && have_trigger_)
      {
        cout<<"which中，从WAIT_TARGET状态切换到GEN_NEW_PATH状态"<<endl;
        changeStateTo(GEN_NEW_PATH, "FSM");
        break;
      }
      else
      {
        goto force_return;
      }
    }

    //===============================3/6 gen_new_path===============================================================
    // ----- 生成新路径：从当前位置算一条全新的全局+局部轨迹；成功就“执行”，失败且已经靠近终点就切下一路点或回“等目标” -----
    case GEN_NEW_PATH:
    {
      cout << "[当前在状态机里]当前状态是：GEN_NEW_PATH" << endl;
      bool success = planFromGlobalPath(10); // 从当前 odom 算一条新路径，最多试 10 次
      if (success)
      {
        changeStateTo(EXEC_PATH, "FSM"); // 成功则进入“执行”
        flag_escape_emergency_ = true;
        publishSwarmPaths(false);
      }
      else
      {
        // 失败时：若已是“预设/全局路径”且当前位置离终点很近，视为到达当前路点
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
            (robot_location_now_fromOdomDirectly_ - end_pt_).norm() < no_replan_thresh_)
        {
          if (wp_id_ < waypoint_index_now_ - 1)
          {
            wp_id_++;                      // 还有下一路点：切到下一路点再规划
            planNextWaypoint(waypoints_array_xyz_[wp_id_]);
          }
          else
          {
            have_target_ = false;           // 已是最后一个路点：清目标与触发，回“等目标”
            have_trigger_ = false;
            changeStateTo(WAIT_TARGET, "FSM");
          }
        }
        else
          changeStateTo(GEN_NEW_PATH, "FSM"); // 否则继续留在本状态，下次再试
      }
      break;
    }

    //===============================4/6 replan_path===============================================================
    // ----- 重规划：从当前轨迹上的“现在”位置再算一条新轨迹；成功就“执行”，失败且靠近终点就下一路点或回“等目标” -----
    case REPLAN_PATH:
    {
      static string state_str[7] = {"STATE_ONE__WAIT_FOR_ODOM", "WAIT_TARGET", "GEN_NEW_PATH", "REPLAN_PATH", "EXEC_PATH", "EMERGENCY_STOP"};
      cout << "[当前状态]: 当前状态是：" << state_str[int(current_state_)] << endl;
      // 从当前路径上的“现在”位置做一次局部重规划；参数 1 为 trial_times（失败时用 poly+rebound 最多重试 1 次）
      if (planFromCurrentPath(1))//规划局部路径
      {
        // 重规划成功：切到 EXEC_PATH 执行新轨迹，并发布本机轨迹给其他无人机
        cout << "[状态切换]: 从 " << state_str[int(current_state_)] << " 转为 " << state_str[int(EXEC_PATH)] << endl;
        changeStateTo(EXEC_PATH, "FSM");
        publishSwarmPaths(false);
      }
      else
      {
        // 重规划失败：若为预设/全局路径模式且当前位置已“靠近当前路点终点”（距离 < no_replan_thresh_），视为到达当前路点
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
            (robot_location_now_fromOdomDirectly_ - end_pt_).norm() < no_replan_thresh_)
        {
          if (wp_id_ < waypoint_index_now_ - 1)
          {
            // 还有下一路点：wp_id_ 加一，并对下一路点做全局规划
            wp_id_++;
            planNextWaypoint(waypoints_array_xyz_[wp_id_]);
          }
          else
          {
            // 已是最后一个路点：清空目标与触发，回到 WAIT_TARGET 等待新目标
            have_target_ = false;
            have_trigger_ = false;
            changeStateTo(WAIT_TARGET, "FSM");
          }
        }
        else
          // 未靠近终点或非多路点模式：保持 REPLAN_PATH，下次 10ms 再试
          changeStateTo(REPLAN_PATH, "FSM");
      }

      break;
    }

    //===============================5/6 exec_path===============================================================
    // ----- 执行路径：看当前走到哪了；够时间/距离就触发“重规划”，快到终点或跑完就下一路点或回“等目标” -----
    case EXEC_PATH:
    {
      cout << "[当前在状态机里]当前状态是：EXEC_PATH" << endl;
      /* determine if need to replan */
      LocalPathData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_path_.evaluateDeBoorT(t_cur);

      // USE_GLOBAL_PATH 向前推进：当前段剩余距离不足时，用 last_pct_path_ + 当前位置 重新取前 7m，补充到后面，再规划
      const double roll_forward_thresh = 3.5;  // 剩余距离小于此值（米）时从 pct_path_unfinished 补充新段
      if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH && have_pct_path_ && !last_pct_path_.poses.empty() &&
          (end_pt_ - pos).norm() < roll_forward_thresh)
      {
        std::vector<Eigen::Vector3d> globalpath_points;
        globalpath_points.reserve(last_pct_path_.poses.size());
        for (const auto &ps : last_pct_path_.poses)
        {
          const auto &p = ps.pose.position;
          Eigen::Vector3d pt(p.x, p.y, p.z);
          if (plan_xy_only_)
            pt(2) = 0.0;
          globalpath_points.push_back(pt);
        }
        if (globalpath_points.size() >= 2)
        {
          Eigen::Vector3d robot_location = robot_location_now_fromOdomDirectly_;
          if (plan_xy_only_)
            robot_location(2) = 0.0;
          std::vector<Eigen::Vector3d> unfinished_points;
          nav_msgs::msg::Path unfinished_path;
          if (buildUnfinishedFromGlobalPath(globalpath_points, robot_location, unfinished_points, &unfinished_path, &last_pct_path_.header) &&
              unfinished_points.size() >= 2)
          {
            const double guide_len = 7.0;
            pct_path_unfinished_pub_->publish(unfinished_path);

            std::vector<Eigen::Vector3d> first_7m_pts;
            double templength = 0.0;
            for (size_t i = 1; i < unfinished_points.size(); ++i)
            {
              first_7m_pts.push_back(unfinished_points[i]);
              templength += (unfinished_points[i] - unfinished_points[i - 1]).norm();
              if (templength >= guide_len - 1e-3)
                break;
            }
            guide_path_7m_withoutFirst5points_fsm_h_.clear();
            if (first_7m_pts.size() > 5)
              guide_path_7m_withoutFirst5points_fsm_h_.insert(guide_path_7m_withoutFirst5points_fsm_h_.end(), first_7m_pts.begin() + 5, first_7m_pts.end());

            waypoint_index_now_ = 0;
            const size_t max_wp = 200;
            templength = 0.0;
            for (size_t i = 1; i < unfinished_points.size() && waypoint_index_now_ < static_cast<int>(max_wp); ++i)
            {
              const Eigen::Vector3d &p = unfinished_points[i];
              waypoints_array_[waypoint_index_now_][0] = p.x();
              waypoints_array_[waypoint_index_now_][1] = p.y();
              waypoints_array_[waypoint_index_now_][2] = plan_xy_only_ ? 0.0 : p.z();
              waypoint_index_now_++;
              templength += (unfinished_points[i] - unfinished_points[i - 1]).norm();
              if (templength >= guide_len - 1e-3)
                break;
            }
            if (waypoint_index_now_ > 0)
            {
              readGivenWps();
              break;  // 本次已触发刷新与规划，下次周期再判
            }
          }
        }
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
          (wp_id_ < waypoint_index_now_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(waypoints_array_xyz_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(waypoints_array_xyz_[wp_id_]);
          }
          /* USE_GLOBAL_PATH: 跑完当前路径后进入 WAIT_TARGET，等待新 /pct_path，不自动循环 */

          changeStateTo(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeStateTo(REPLAN_PATH, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeStateTo(REPLAN_PATH, "FSM");
      }

      break;
    }

    //===============================6/6 emergency_stop===============================================================
    // ----- 紧急停：先发一条“原地停”的轨迹；若开了 fail_safe 且速度下来了就尝试回到“生成新路径” -----
    case EMERGENCY_STOP:
    {
      cout << "[当前在状态机里]当前状态是：EMERGENCY_STOP" << endl;
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(robot_location_now_fromOdomDirectly_);
      }
      else
      {
        if (enable_fail_safe_ && robot_vel_now_fromOdomDirectly_.norm() < 0.1)
          changeStateTo(GEN_NEW_PATH, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    // ----- 发一次调试/可视化用的 data_disp，然后从 force_return 出去 -----
    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // ----- 本次回调结束，把定时器重新开起来，下次 10ms 后再进这个函数 -----
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      exec_timer_->reset();
    }
  }

  /**
   * 基于全局路径/路点，从当前机器人位姿（odom）作为起点规划一条局部轨迹。
   * 与 planFromCurrentPath 的区别：本函数起点为“当前 odom 位置”，后者为“当前执行轨迹上 t_cur 时刻的位姿”。
   * @param trial_times 最多尝试规划的次数，失败则依次重试（可配合多项式/随机初始化增加成功率）
   * @return 成功生成可行局部路径返回 true，否则 false
   */
  bool classEGOPlannerStateMachine::planFromGlobalPath(const int trial_times /*=1*/) // zx-todo
  {
    // ---------- 以当前里程计位姿、速度作为规划的起点，加速度置零 ----------
    start_pt_ = robot_location_now_fromOdomDirectly_;
    start_vel_ = robot_vel_now_fromOdomDirectly_;
    start_acc_.setZero();
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    // ---------- 是否使用随机多项式初始化：首次进入当前状态不用随机，后续重试时启用以增加多样性 ----------
    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    // ---------- 最多尝试 trial_times 次，任一次 plan7mLocalPPath_prepareAndDoit 成功即返回 true ----------
    for (int i = 0; i < trial_times; i++)
    {
      if (plan7mLocalPPath_prepareAndDoit(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  /**
   * 从当前正在执行的轨迹上“现在”时刻的位置/速度/加速度作为起点，重新规划一条局部路径。
   * 规划采用三级回退：先不用多项式初始化 → 再用多项式初始化 → 最后用多项式+随机多项式轨迹，最多重试 trial_times 次。
   */
  bool classEGOPlannerStateMachine::planFromCurrentPath(const int trial_times /*=1*/)
  {
    LocalPathData *info = &planner_manager_->local_data_;
    auto time_now = rclcpp::Clock().now();
    double t_cur = (time_now - info->start_time_).seconds();

    // ---------- 从当前轨迹取 t_cur 时刻的位姿、速度、加速度，作为重规划的起点 ----------
    start_pt_ = info->position_path_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_path_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_path_.evaluateDeBoorT(t_cur);
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    // ---------- 第一级：不启用多项式初始化、不启用随机多项式 (flag_use_poly_init=false, flag_randomPolyTraj=false) ----------
    bool success = plan7mLocalPPath_prepareAndDoit(false, false);
    if(success)
    {
      cout << "第一级：不启用多项式初始化、不启用随机多项式 (flag_use_poly_init=false, flag_randomPolyTraj=false) 规划成功" << endl;
      return true;
    }
    if (!success)
    {
      // ---------- 第二级：启用多项式初始化，不启用随机多项式 ----------
      success = plan7mLocalPPath_prepareAndDoit(true, false);
      if(success)
      {
        cout << "第二级：启用多项式初始化，不启用随机多项式 (flag_use_poly_init=true, flag_randomPolyTraj=false) 规划成功" << endl;
        return true;
      }
      if (!success)
      {
        // ---------- 第三级：启用多项式初始化 + 随机多项式轨迹，最多重试 trial_times 次 ----------
        for (int i = 0; i < trial_times; i++)
        {
          success = plan7mLocalPPath_prepareAndDoit(true, true);
          if (success)
          {
            cout << "第三级：启用多项式初始化 + 随机多项式轨迹，最多重试 trial_times 次 规划成功" << endl;
            return true;
          }
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * 安全检测回调：每 100ms 由定时器调用一次。检查 (1) 深度/传感器是否超时 (2) 当前轨迹是否与障碍或其它机发生碰撞；
   * 若碰撞则尝试从当前轨迹重规划，失败则根据时间紧急程度切到 EMERGENCY_STOP 或 REPLAN_PATH。
   */
  void classEGOPlannerStateMachine::autofunction_checkStoneCallback_every100ms()
  {
    LocalPathData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // ---------- 未在执行路径或轨迹尚未有效时直接返回，不做碰撞检测 ----------
    if (current_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    // ---------- 深度/传感器超时：视为丢失深度，立即紧急停并关闭 fail_safe ----------
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeStateTo(EMERGENCY_STOP, "SAFETY");
    }

    // ---------- 若地图自上次检查以来完全未更新，在当前 EXEC_PATH 下直接沿用原局部路径，不做重新碰撞检测 ----------
    // 这样可以实现“障碍物位置与大小都未变化时，局部路径保持不变，机器狗沿当前轨迹继续前进”
    int cur_grid_update_num = map->getUpdateNum();
    if (current_state_ == EXEC_PATH && last_grid_update_num_ == cur_grid_update_num)
    {
      return;
    }
    last_grid_update_num_ = cur_grid_update_num;

    // ---------- 轨迹碰撞检测：从当前时刻 t_cur 起，沿轨迹以 0.01s 步长采样，检查每点是否占据障碍或与其它机过近 ----------
    constexpr double time_step = 0.01;
    double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();
    Eigen::Vector3d p_cur = info->position_path_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();  // 与其它机的最小间隔
    double t_cur_global = rclcpp::Clock().now().seconds();

    // 仅检查轨迹前 2/3 段的有效性：若 t_cur 还在前 2/3，则只检查到 t_2_3 为止，避免对尚未执行到的后段误判
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      bool occ = false;
      // 地图障碍：当前轨迹上 t 时刻位置是否在膨胀占据栅格内
      occ |= map->getInflateOccupancy(info->position_path_.evaluateDeBoorT(t));

      // 集群：与其它机的预测位置距离小于 CLEARANCE 则视为碰撞
      for (size_t id = 0; id < planner_manager_->swarm_paths_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_paths_buf_.at(id).dog_id != (int)id) || (planner_manager_->swarm_paths_buf_.at(id).dog_id == planner_manager_->pp_.dog_id))
          continue;

        double t_X = t_cur_global - planner_manager_->swarm_paths_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_paths_buf_.at(id).position_path_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();
        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {
        // 发现碰撞：先尝试从当前轨迹起点重规划一条新路径
        if (planFromCurrentPath())
        {
          changeStateTo(EXEC_PATH, "SAFETY");
          publishSwarmPaths(false);
          return;
        }
        // 重规划失败：若碰撞发生在近端（t - t_cur < emergency_time_）则紧急停，否则触发重规划状态
        if (t - t_cur < emergency_time_)
        {
          RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
          changeStateTo(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(), "current path in collision, replan.");
          changeStateTo(REPLAN_PATH, "SAFETY");
        }
        return;
      }
    }
  }

  /**
   * 调用规划器生成一条从 start_pt_ 到局部目标 local_target_pt_ 的 B 样条局部路径，并发布给轨迹执行与可视化。
   * @param flag_use_poly_init 是否使用多项式初始化轨迹（重规划/重试时多为 true）
   * @param flag_randomPolyTraj 是否在初始化时加入随机多项式轨迹以增加多样性，提高在复杂障碍下的成功率
   * @return 规划与时间重分配均成功且轨迹无碰撞返回 true，否则 false
   */
  bool classEGOPlannerStateMachine::plan7mLocalPPath_prepareAndDoit(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // ---------- 根据最新 odometry 设置规划起点 start_pt_、start_vel_、start_acc_ ----------
    getNowLocationAndVel();
    // ---------- 根据当前目标/路点更新局部目标 local_target_pt_、local_target_vel_（供 plan7mLocalPath 使用）----------
    get7mEndPoint();//执行此函数，获得了local_target_pt_, local_target_vel_

    // ---------- 调用规划器：起点 (start_pt_, start_vel_, start_acc_)，终点 (local_target_pt_, local_target_vel_)；结果写入 planner_manager_->local_data_ ----------
    bool plan_and_refine_success =
        planner_manager_->plan7mLocalPath(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success;
    if (plan_and_refine_success)
    {
      cout << "（规划与时间重分配成功，轨迹可行）" << endl;
    }
    else
    {
      cout << "（规划或时间重分配失败，轨迹不可行）" << endl;
    }

    // ---------- 规划成功时：将 local_data_ 中的 B 样条转为 ROS 消息并发布、可视化 ----------
    if (plan_and_refine_success)
    {
      auto info = &planner_manager_->local_data_;

      path_tools::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.dog_id = planner_manager_->pp_.dog_id;
      bspline.path_id = info->path_id_;

      // 控制点：从 position_path_ 的矩阵按列转为 geometry_msgs::Point 数组
      Eigen::MatrixXd pos_pts = info->position_path_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 节点向量：定义 B 样条基函数与参数化
      Eigen::VectorXd knots = info->position_path_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. 发布给 path_server / 轨迹执行节点 */
      bspline_pub_->publish(bspline);

      /* 2. 集群中下一架无人机的路径由 publishSwarmPaths 等逻辑单独发布 */

      /* 3. 发布到 RViz 等可视化 */
      visualization_->displayOptimalList(info->position_path_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void classEGOPlannerStateMachine::publishSwarmPaths(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    path_tools::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.dog_id = planner_manager_->pp_.dog_id;
    bspline.path_id = info->path_id_;

    Eigen::MatrixXd pos_pts = info->position_path_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_path_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.dog_id_from = planner_manager_->pp_.dog_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.path.size() == planner_manager_->pp_.dog_id + 1)
      {
        multi_bspline_msgs_buf_.path.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.path.size() == planner_manager_->pp_.dog_id)
      {
        multi_bspline_msgs_buf_.path.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong path nums and dog_id pair!!! path.size()=%d, dog_id=%d", (int)multi_bspline_msgs_buf_.path.size(), planner_manager_->pp_.dog_id);
        // return plan_and_refine_success;
      }
      // swarm_paths_pub_.publish(multi_bspline_msgs_buf_);
      swarm_paths_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool classEGOPlannerStateMachine::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (plan_xy_only_)
      stop_pos(2) = 0.0;
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish path */
    path_tools::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.path_id = info->path_id_;

    Eigen::MatrixXd pos_pts = info->position_path_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_path_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void classEGOPlannerStateMachine::getNowLocationAndVel()
  {
    // robot_location_now_fromOdomDirectly_、robot_vel_now_fromOdomDirectly_ 由 autoFunction_GetOdometry 根据 /odometry（订阅名 odom_world）话题持续更新，此处即用该最新值作为规划起点
    start_pt_ = robot_location_now_fromOdomDirectly_;
    start_vel_ = robot_vel_now_fromOdomDirectly_;
    start_acc_.setZero();
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }
  }

  void classEGOPlannerStateMachine::get7mEndPoint()
  {
    // USE_GLOBAL_PATH 模式：引导段取自 /pct_path_unfinished 前 7m 且去掉最前5个点的 guide_path_7m_withoutFirst5points_fsm_h_
    if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH && !guide_path_7m_withoutFirst5points_fsm_h_.empty())
    {
      planner_manager_->copy7mPath(guide_path_7m_withoutFirst5points_fsm_h_);// 函数体：{ guide_path_7m_withoutFirst5points_manager_h_ = seg; }
      local_target_pt_ = guide_path_7m_withoutFirst5points_fsm_h_.back();
      local_target_vel_ = Eigen::Vector3d::Zero();
      if (plan_xy_only_)
      {
        local_target_pt_(2) = 0.0;
        local_target_vel_(2) = 0.0;
      }
      return;
    }

    //如果快到终点了，guide_path_7m_withoutFirst5points_fsm_h_为空
    const double t_step = 0.05;
    GlobalPathData &global_data = planner_manager_->global_path_afterCalculate_;
    const double global_duration = global_data.global_duration_;
    double dist_min = 1e9;
    double t_closest = global_data.last_progress_time_;
    for (double t = 0.0; t <= global_duration + 1e-6; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_path_afterCalculate_.getPosition(t);
      if (plan_xy_only_)
        pos_t(2) = 0.0;
      double dist = (pos_t - start_pt_).norm();

      if (dist < dist_min)
      {
        dist_min = dist;
        t_closest = t;
      }

    }
    global_data.last_progress_time_ = t_closest;

    // 从最近点开始向前截取约 7 米的一段作为局部规划的目标路径/引导边界
    const double segment_length = 7.0;
    std::vector<Eigen::Vector3d> segment_pts;
    Eigen::Vector3d p0 = global_data.getPosition(t_closest);
    if (plan_xy_only_)
      p0(2) = 0.0;
    segment_pts.push_back(p0);
    double arc = 0.0;
    double t_cur = t_closest;
    double t_end = t_closest;
    Eigen::Vector3d pos_prev = segment_pts.front();
    while (arc < segment_length && t_cur < global_duration - 1e-6)
    {
      t_cur = std::min(t_cur + t_step, global_duration);
      Eigen::Vector3d pos_cur = global_data.getPosition(t_cur);
      if (plan_xy_only_)
        pos_cur(2) = 0.0;
      arc += (pos_cur - pos_prev).norm();
      pos_prev = pos_cur;
      segment_pts.push_back(pos_cur);
      t_end = t_cur;
    }
    if (segment_pts.empty())
    {
      local_target_pt_ = end_pt_;
      local_target_vel_ = Eigen::Vector3d::Zero();
      if (plan_xy_only_)
      { local_target_pt_(2) = 0.0; local_target_vel_(2) = 0.0; }
      planner_manager_->copy7mPath(std::vector<Eigen::Vector3d>());
      return;
    }
    local_target_pt_ = segment_pts.back();
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
      local_target_vel_ = Eigen::Vector3d::Zero();
    else
    {
      local_target_vel_ = global_data.getVelocity(t_end);
      if (plan_xy_only_)
        local_target_vel_(2) = 0.0;
    }
    if (plan_xy_only_)
      local_target_pt_(2) = 0.0;
    planner_manager_->copy7mPath(segment_pts);
  }

} // namespace ego_planner
