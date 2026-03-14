
#include <ego_planner/ego_replan_fsm.h>
#include <limits>

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    have_pct_path_ = false;
    exec_state_ = FSM_EXEC_STATE::INIT;
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

    node_->get_parameter("fsm/egoplanner_input_point_or_path", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/path_ahead_time", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/plan_xy_only", plan_xy_only_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverPathToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_paths");
      swarm_paths_sub_ = node_->create_subscription<path_tools::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const path_tools::msg::MultiBsplines> &msg)
          {
            this->swarmPathsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_paths");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_paths");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_paths");
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

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = plan_xy_only_ ? 0.0 : waypoints_[i][2];
    }

    wp_id_ = 0;

    // USE_GLOBAL_PATH 且多路点：第一个目标点 = “最近点沿路径前进约2m处”，再沿路径到终点
    if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH && waypoint_num_ > 1)
    {
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

      // 1) 找路径上离当前位置最近的点 join_pt（位于 wps_[seg_idx] -> wps_[seg_idx+1] 的线段上）
      Eigen::Vector3d join_pt = wps_[0];
      double join_t = 0.0;
      double best_d2 = 1e30;
      int seg_idx = 0;
      Eigen::Vector3d cand;
      double cand_t = 0.0;
      for (int i = 0; i < waypoint_num_ - 1; i++)
      {
        double d2 = closestOnSegment(odom_pos_, wps_[i], wps_[i + 1], cand, cand_t);
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

      // 2) 顺着路径方向，从 join_pt 往前走约 2m，得到第一个目标点 first_pt
      const double forward_dist = 2.0;
      double remain = forward_dist;
      Eigen::Vector3d first_pt = join_pt;

      // 从当前线段的 join_t 位置开始往前推
      Eigen::Vector3d a = wps_[seg_idx];
      Eigen::Vector3d b = wps_[seg_idx + 1];
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
        while (remain > 1e-6 && j < waypoint_num_ - 1)
        {
          Eigen::Vector3d p0 = wps_[j];
          Eigen::Vector3d p1 = wps_[j + 1];
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

      // 3) 用 first_pt 作为新的第一个 waypoint，并从其所在段之后继续到终点
      std::vector<Eigen::Vector3d> new_wps;
      new_wps.reserve(waypoint_num_ + 1);
      new_wps.push_back(first_pt);
      for (int i = seg_idx + 1; i < waypoint_num_; i++)
      {
        if ((wps_[i] - first_pt).norm() > 1e-3)
          new_wps.push_back(wps_[i]);
      }
      wps_ = new_wps;
      waypoint_num_ = (int)wps_.size();
      wp_id_ = 0;

      // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化（使用更新后的 wps_）
      for (size_t i = 0; i < (size_t)waypoint_num_; i++)
      {
        visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      std::vector<Eigen::Vector3d> waypoints_vec;
      for (int i = 0; i < waypoint_num_; i++)
        waypoints_vec.push_back(wps_[i]);

      bool success = planner_manager_->planGlobalPathWaypoints(
          odom_pos_,
          [&]() -> Eigen::Vector3d {
            // 用“沿路径前进方向”的速度约束生成全局参考轨迹，避免因 odom_vel_ 横向/反向导致 min-snap 轨迹折返
            Eigen::Vector3d v = odom_vel_;
            if (plan_xy_only_)
              v(2) = 0.0;
            Eigen::Vector3d dir = first_pt - odom_pos_;
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

      if (success)
      {
        end_pt_ = wps_[wp_id_];
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> global_path(i_end);
        for (int i = 0; i < i_end; i++)
        {
          global_path[i] = planner_manager_->global_data_.global_path_.evaluate(i * step_size_t);
          if (plan_xy_only_)
            global_path[i](2) = 0.0;
        }
        visualization_->displayGlobalPathList(global_path, 0.1, 0);

        if (exec_state_ == WAIT_TARGET)
          changeFSMExecState(GEN_NEW_PATH, "TRIG");
        else
          changeFSMExecState(REPLAN_PATH, "TRIG");
        return;
      }
      // 全路径规划失败时退化为只规划到第一个路点
    }

    // 非 USE_GLOBAL_PATH：用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 单路点或非 USE_GLOBAL_PATH 或上面全路径规划失败：只规划到第一个路点
    planNextWaypoint(wps_[wp_id_]);
  }

  // /pct_path 话题回调：
  // 1）接收外部给的一整条“全局参考路径”（密集点，不再做稀疏采样）；
  // 2）结合当前里程计位置 a，切掉“已经走过的前半段”，得到从 a 开始的未走完路径 pct_path_unfinished 并发布到 /pct_path_unfinished；
  // 3）在 pct_path_unfinished 里，从 a 开始累计大约 7m 的一小段，作为局部规划的引导路径喂给 EGO Planner。
  void EGOReplanFSM::pctPathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg)
  {
    // 如果此时还没有里程计，就没法知道“当前位置 a 在路径上的什么位置”，只能先忽略
    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(), "/pct_path received but odom not ready yet.");
      return;
    }

    // 1）安全性检查：如果消息里一个点都没有，直接忽略，不进行后续处理
    if (msg->poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Received empty /pct_path, ignore.");
      return;
    }

    // 2）先把原始 /pct_path 中的所有点转成 Eigen 向量，保留“密集路径”（不做降采样）
    std::vector<Eigen::Vector3d> raw_pts;
    raw_pts.reserve(msg->poses.size());
    for (const auto &ps : msg->poses)
    {
      const auto &p = ps.pose.position;
      Eigen::Vector3d pt(p.x, p.y, p.z);
      if (plan_xy_only_)
        pt(2) = 0.0;
      raw_pts.push_back(pt);
    }
    if (raw_pts.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "pct_path has less than 2 points, ignore.");
      return;
    }

    // 当前里程计位置 a（如只规划 XY，则把 z 置 0）
    Eigen::Vector3d a = odom_pos_;
    if (plan_xy_only_)
      a(2) = 0.0;

    // 3）在原始路径 raw_pts 中找一个“离 a 最近的点 b”，作为当前所在路径上的参考点
    int b_idx = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(raw_pts.size()); ++i)
    {
      double d2 = (raw_pts[i] - a).squaredNorm();
      if (d2 < best_d2)
      {
        best_d2 = d2;
        b_idx = i;
      }
    }
    Eigen::Vector3d b = raw_pts[b_idx];

    // 计算 a 到 b 的直线距离 ab，用它来决定“向前再走多远找到点 c”
    double ab = (b - a).norm();

    // 4）从 b 开始，沿着路径方向向前“累积路径弧长”，走大约 ab 的长度，找到路径上的点 c
    Eigen::Vector3d c = b;
    int c_seg_idx = b_idx; // c 所在的线段起点索引
    if (ab > 1e-3 && b_idx < static_cast<int>(raw_pts.size()) - 1)
    {
      double remain = ab;
      Eigen::Vector3d cur = b;
      bool found_c = false;
      for (int i = b_idx; i < static_cast<int>(raw_pts.size()) - 1; ++i)
      {
        Eigen::Vector3d next = raw_pts[i + 1];
        double seg_len = (next - cur).norm();
        if (seg_len < 1e-6)
        {
          cur = next;
          continue;
        }
        if (remain <= seg_len)
        {
          double ratio = remain / seg_len;
          c = cur + ratio * (next - cur);
          c_seg_idx = i;
          found_c = true;
          break;
        }
        else
        {
          remain -= seg_len;
          cur = next;
        }
      }
      // 如果到路径末尾都没走完 remain，就把 c 放在最后一个点
      if (!found_c)
      {
        c = raw_pts.back();
        c_seg_idx = static_cast<int>(raw_pts.size()) - 2;
      }
    }

    // 5）构造“从 a 走到 c，再接上 c 之后尚未走完的整条路径”的未完成路径 pct_path_unfinished
    std::vector<Eigen::Vector3d> unfinished_pts;
    unfinished_pts.reserve(raw_pts.size());

    // 5.1）先在 a 到 c 之间，每隔约 0.1m 取一个点；如果 ac 很短，则只取 a 和 c
    Eigen::Vector3d ac_vec = c - a;
    double ac_len = ac_vec.norm();
    const double ds = 0.1; // 10cm 步长

    unfinished_pts.push_back(a);
    if (ac_len > 1e-3)
    {
      int steps = static_cast<int>(std::floor(ac_len / ds));
      Eigen::Vector3d dir = ac_vec / ac_len;
      for (int i = 1; i < steps; ++i)
      {
        double dist = i * ds;
        unfinished_pts.push_back(a + dist * dir);
      }
      // 把 c 作为这一段的终点
      unfinished_pts.push_back(c);
    }

    // 5.2）再把 c 之后的路径原样拼接上去（跳过 c 所在线段的起点，避免重复）
    for (int i = c_seg_idx + 1; i < static_cast<int>(raw_pts.size()); ++i)
    {
      unfinished_pts.push_back(raw_pts[i]);
    }

    // 5.3）发布 /pct_path_unfinished，方便 RViz 中查看“从当前位置开始还没走完的全局参考路径”
    nav_msgs::msg::Path unfinished_msg;
    unfinished_msg.header = msg->header;
    unfinished_msg.poses.resize(unfinished_pts.size());
    for (size_t i = 0; i < unfinished_pts.size(); ++i)
    {
      auto &ps = unfinished_msg.poses[i];
      ps.header = msg->header;
      ps.pose.position.x = unfinished_pts[i].x();
      ps.pose.position.y = unfinished_pts[i].y();
      ps.pose.position.z = unfinished_pts[i].z();
      // 姿态这里不做精细处理，简单置为单位四元数即可
      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;
    }
    pct_path_unfinished_pub_->publish(unfinished_msg);

    // 6）从 pct_path_unfinished（即 unfinished_pts）中按顺序全取前 7m 的点，去掉最前两个点，作为局部规划引导段送给 EGO Planner
    const double guide_len = 7.0;  // 单位 m
    std::vector<Eigen::Vector3d> first_7m_pts;
    first_7m_pts.reserve(unfinished_pts.size());
    double cum = 0.0;
    for (size_t i = 1; i < unfinished_pts.size(); ++i)
    {
      first_7m_pts.push_back(unfinished_pts[i]);
      if (i > 0)
      {
        cum += (unfinished_pts[i] - unfinished_pts[i - 1]).norm();
        if (cum >= guide_len - 1e-6)
          break;
      }
    }
    // 去掉最前面的 5 个点后送给 EGO Planner
    pct_guide_segment_.clear();
    if (first_7m_pts.size() > 5)
    {
      pct_guide_segment_.insert(pct_guide_segment_.end(), first_7m_pts.begin() + 5, first_7m_pts.end());
    }

    // 7）waypoints_ 仍用于全局多路点规划：取前 7m 填 waypoints_，供 readGivenWps 使用
    waypoint_num_ = 0;
    const size_t max_wp = 200;
    cum = 0.0;
    for (size_t i = 0; i < unfinished_pts.size() && waypoint_num_ < static_cast<int>(max_wp); ++i)
    {
      const Eigen::Vector3d &p = unfinished_pts[i];
      waypoints_[waypoint_num_][0] = p.x();
      waypoints_[waypoint_num_][1] = p.y();
      waypoints_[waypoint_num_][2] = plan_xy_only_ ? 0.0 : p.z();
      waypoint_num_++;
      if (i > 0)
      {
        cum += (unfinished_pts[i] - unfinished_pts[i - 1]).norm();
        if (cum >= guide_len - 1e-3)
          break;
      }
    }
    if (waypoint_num_ <= 0)
    {
      RCLCPP_WARN(node_->get_logger(), "No valid waypoints generated from /pct_path_unfinished.");
      return;
    }

    // 标记“已经有一条参考路径了”，并把它当作一次“开始规划”的触发信号
    have_pct_path_ = true;
    have_trigger_ = true; /* USE_GLOBAL_PATH 下用收到路径作为触发 */

    // 8）把采样好的 waypoints_ 转换成内部的 wps_ 向量，并调用原有多路点规划逻辑
    readGivenWps();
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    Eigen::Vector3d wp = next_wp;
    if (plan_xy_only_)
      wp(2) = 0.0;
    bool success = false;
    success = planner_manager_->planGlobalPath(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> global_path(i_end);
      for (int i = 0; i < i_end; i++)
      {
        global_path[i] = planner_manager_->global_data_.global_path_.evaluate(i * step_size_t);
        if (plan_xy_only_)
          global_path[i](2) = 0.0;
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_PATH, "TRIG");
      else
      {
        /* 已在 executor 内，不再阻塞 spin_some，避免 "Node has already been added to an executor" */
        changeFSMExecState(REPLAN_PATH, "TRIG");
      }

      visualization_->displayGlobalPathList(global_path, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = plan_xy_only_ ? 0.0 : msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = plan_xy_only_ ? 0.0 : msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const path_tools::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_paths_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_paths_buf_.size(); i <= id; i++)
      {
        OnePathDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_paths_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_paths_buf_[id].drone_id = -1;
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

    planner_manager_->swarm_paths_buf_[id].drone_id = id;

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
      changeFSMExecState(REPLAN_PATH, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmPathsCallback(const std::shared_ptr<const path_tools::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.path.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmPathsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->path.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->path.size()=%d, msg->drone_id_from+1=%d", (int)msg->path.size(), msg->drone_id_from + 1);
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
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_paths_buf_[i].drone_id = -1;
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

      planner_manager_->swarm_paths_buf_[i].drone_id = i;

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

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_PATH", "REPLAN_PATH", "EXEC_PATH", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_PATH", "REPLAN_PATH", "EXEC_PATH", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalPath(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_PATH, "FSM");

            publishSwarmPaths(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_PATH:
    {

      bool success = planFromGlobalPath(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_PATH, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmPaths(false);
      }
      else
      {
        /* “Close to goal” 时规划会失败，视为已到达当前路点，切下一路点或结束 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
            (odom_pos_ - end_pt_).norm() < no_replan_thresh_)
        {
          if (wp_id_ < waypoint_num_ - 1)
          {
            wp_id_++;
            planNextWaypoint(wps_[wp_id_]);
          }
          else
          {
            have_target_ = false;
            have_trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
        }
        else
          changeFSMExecState(GEN_NEW_PATH, "FSM");
      }
      break;
    }

    case REPLAN_PATH:
    {

      if (planFromCurrentPath(1))
      {
        changeFSMExecState(EXEC_PATH, "FSM");
        publishSwarmPaths(false);
      }
      else
      {
        /* “Close to goal” 时规划会失败，视为已到达当前路点，切下一路点或结束 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
            (odom_pos_ - end_pt_).norm() < no_replan_thresh_)
        {
          if (wp_id_ < waypoint_num_ - 1)
          {
            wp_id_++;
            planNextWaypoint(wps_[wp_id_]);
          }
          else
          {
            have_target_ = false;
            have_trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
        }
        else
          changeFSMExecState(REPLAN_PATH, "FSM");
      }

      break;
    }

    case EXEC_PATH:
    {
      /* determine if need to replan */
      LocalPathData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_path_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::USE_GLOBAL_PATH) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
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
            planNextWaypoint(wps_[wp_id_]);
          }
          /* USE_GLOBAL_PATH: 跑完当前路径后进入 WAIT_TARGET，等待新 /pct_path，不自动循环 */

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_PATH, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_PATH, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_PATH, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalPath(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callPlanLocalPath(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentPath(const int trial_times /*=1*/)
  {

    LocalPathData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_path_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_path_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_path_.evaluateDeBoorT(t_cur);
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    bool success = callPlanLocalPath(false, false);

    if (!success)
    {
      success = callPlanLocalPath(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callPlanLocalPath(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalPathData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_path_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = rclcpp::Clock().now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_path_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_paths_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_paths_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_paths_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

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

        if (planFromCurrentPath()) // Make a chance
        {
          changeFSMExecState(EXEC_PATH, "SAFETY");
          publishSwarmPaths(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current path in collision, replan.");
            changeFSMExecState(REPLAN_PATH, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callPlanLocalPath(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->planLocalPath(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success;
    if (plan_and_refine_success)
    {
      cout << "（规划与时间重分配成功，轨迹可行且未碰撞）" << endl;
    }
    else
    {
      cout << "（规划或时间重分配失败，轨迹不可行或发生碰撞）" << endl;
    }

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

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

      /* 1. publish path to path_server */
      bspline_pub_->publish(bspline);

      /* 2. publish path to the next drone of swarm */

      /* 3. publish path for visualization */
      visualization_->displayOptimalList(info->position_path_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmPaths(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    path_tools::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
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
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.path.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.path.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.path.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.path.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong path nums and drone_id pair!!! path.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.path.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_paths_pub_.publish(multi_bspline_msgs_buf_);
      swarm_paths_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
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

  void EGOReplanFSM::getLocalTarget()
  {
    // USE_GLOBAL_PATH 模式：引导段取自 /pct_path_unfinished 前 7m 且去掉最前两点的 pct_guide_segment_
    if (target_type_ == TARGET_TYPE::USE_GLOBAL_PATH && !pct_guide_segment_.empty())
    {
      planner_manager_->setLocalGuideSegment(pct_guide_segment_);
      local_target_pt_ = pct_guide_segment_.back();
      local_target_vel_ = Eigen::Vector3d::Zero();
      if (plan_xy_only_)
      {
        local_target_pt_(2) = 0.0;
        local_target_vel_(2) = 0.0;
      }
      return;
    }

    const double t_step = 0.05;
    GlobalPathData &global_data = planner_manager_->global_data_;
    const double global_duration = global_data.global_duration_;
    double dist_min = 1e9;
    double t_closest = global_data.last_progress_time_;
    for (double t = 0.0; t <= global_duration + 1e-6; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
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
      planner_manager_->setLocalGuideSegment(std::vector<Eigen::Vector3d>());
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
    planner_manager_->setLocalGuideSegment(segment_pts);
  }

} // namespace ego_planner
