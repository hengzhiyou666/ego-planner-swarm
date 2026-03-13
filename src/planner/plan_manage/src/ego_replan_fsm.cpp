
#include <ego_planner/ego_replan_fsm.h>

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
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/plan_xy_only", false);

    node_->get_parameter("fsm/egoplanner_input_point_or_path", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
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

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
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
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

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
          "/traj_start_trigger",
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
    else if (target_type_ == TARGET_TYPE::REFENCE_PATH)
    {
      pct_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "/pct_path",
          1,
          [this](const std::shared_ptr<const nav_msgs::msg::Path> &msg)
          {
            this->pctPathCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "REFENCE_PATH mode: waiting for /pct_path and odom.");
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

    // REFENCE_PATH 且多路点：第一个目标点 = “最近点沿路径前进约2m处”，再沿路径到终点
    if (target_type_ == TARGET_TYPE::REFENCE_PATH && waypoint_num_ > 1)
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

      bool success = planner_manager_->planGlobalTrajWaypoints(
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
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
          gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
          if (plan_xy_only_)
            gloabl_traj[i](2) = 0.0;
        }
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

        if (exec_state_ == WAIT_TARGET)
          changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        else
          changeFSMExecState(REPLAN_TRAJ, "TRIG");
        return;
      }
      // 全路径规划失败时退化为只规划到第一个路点
    }

    // 非 REFENCE_PATH：用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 单路点或非 REFENCE_PATH 或上面全路径规划失败：只规划到第一个路点
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::pctPathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg)
  {
    if (msg->poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Received empty /pct_path, ignore.");
      return;
    }

    // 将 /pct_path 转换为间隔约 5m 的离散路点
    waypoint_num_ = 0;

    const size_t max_wp = 50;

    Eigen::Vector3d last_pt;
    double accumulated_dist = 0.0;

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      const auto &pose = msg->poses[i].pose;
      Eigen::Vector3d pt(pose.position.x, pose.position.y, pose.position.z);

      if (i == 0)
      {
        // 第一个点直接作为起点；plan_xy_only 时仅用 xy，z 强制为 0
        waypoints_[waypoint_num_][0] = pt.x();
        waypoints_[waypoint_num_][1] = pt.y();
        waypoints_[waypoint_num_][2] = plan_xy_only_ ? 0.0 : pt.z();
        last_pt = pt;
        waypoint_num_++;

        if (waypoint_num_ >= (int)max_wp)
          break;

        continue;
      }

      double dist = (pt - last_pt).norm();
      accumulated_dist += dist;
      last_pt = pt;

      // 每累计约 5m 取一个路点，保证“前进 5m 一个局部目标”；plan_xy_only 时 z 强制为 0
      if (accumulated_dist >= 5.0 - 1e-3)
      {
        waypoints_[waypoint_num_][0] = pt.x();
        waypoints_[waypoint_num_][1] = pt.y();
        waypoints_[waypoint_num_][2] = plan_xy_only_ ? 0.0 : pt.z();

        waypoint_num_++;
        accumulated_dist = 0.0;

        if (waypoint_num_ >= (int)max_wp)
          break;
      }
    }

    if (waypoint_num_ <= 0)
    {
      RCLCPP_WARN(node_->get_logger(), "No valid waypoints generated from /pct_path.");
      return;
    }

    have_pct_path_ = true;
    have_trigger_ = true; /* REFENCE_PATH 下用收到路径作为触发 */

    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(), "/pct_path received but odom not ready yet.");
      return;
    }

    // 利用生成的路点序列，复用原有 readGivenWps + planNextWaypoint 逻辑
    readGivenWps();
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    Eigen::Vector3d wp = next_wp;
    if (plan_xy_only_)
      wp(2) = 0.0;
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        if (plan_xy_only_)
          gloabl_traj[i](2) = 0.0;
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        /* 已在 executor 内，不再阻塞 spin_some，避免 "Node has already been added to an executor" */
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
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

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
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
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
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

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
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
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

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
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
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

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        /* “Close to goal” 时规划会失败，视为已到达当前路点，切下一路点或结束 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::REFENCE_PATH) &&
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
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        /* “Close to goal” 时规划会失败，视为已到达当前路点，切下一路点或结束 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::REFENCE_PATH) &&
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
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET || target_type_ == TARGET_TYPE::REFENCE_PATH) &&
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
          /* REFENCE_PATH: 跑完当前路径后进入 WAIT_TARGET，等待新 /pct_path，不自动循环 */

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
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
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
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

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
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
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    if (plan_xy_only_)
    {
      start_pt_(2) = 0.0;
      start_vel_(2) = 0.0;
      start_acc_(2) = 0.0;
    }

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
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

    LocalTrajData *info = &planner_manager_->local_data_;
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

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = rclcpp::Clock().now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
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
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (plan_xy_only_)
      stop_pos(2) = 0.0;
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
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
    const double t_step = 0.05;
    GlobalTrajData &global_data = planner_manager_->global_data_;
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
