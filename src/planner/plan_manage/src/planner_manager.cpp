// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  // 初始化规划模块：从 node 读取 manager 参数，创建并初始化栅格地图、B 样条优化器、A*、以及可视化句柄
  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/path_ahead_time", 5.0);
    node->declare_parameter("manager/try_more_paths_and_choose_best", false);
    node->declare_parameter("manager/dog_id", -1);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/path_ahead_time", pp_.planning_horizen_);
    node->get_parameter("manager/try_more_paths_and_choose_best", pp_.try_more_paths_and_choose_best);
    node->get_parameter("manager/dog_id", pp_.dog_id);

    local_data_.path_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // 规划局部路径：根据当前状态与局部目标，生成一条满足动力学约束的 B 样条轨迹（小白：算一段从当前点到前方目标点的可行路径）
  // flag_polyInit：是否用多项式重新生成初始路径；flag_randomPolyTraj：是否在初始路径中插入随机点
  bool EGOPlannerManager::plan7mLocalPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    // ==================== 模块 0：入口与前置检查 ====================
    rclcpp::Time t_plan7m_begin = rclcpp::Clock().now();
    static int count = 0;
    printf("\033[47;30m\n[robot replan 「核心算法」第%d次规划局部路径，开始规划...plan7mLocalPath（）]==============================================\033[0m\n", count++);

    double distance_to_goal = (start_pt - local_target_pt).norm();
    if (distance_to_goal < 0.2)
    {
      cout << "到达目的地附近，reached the destination, distance_to_goal: " << distance_to_goal << endl;
      continous_failures_count_++;
      printf("\033[42m[plan7mLocalPath] 执行耗时: %.3f ms\033[0m\n", (rclcpp::Clock().now() - t_plan7m_begin).seconds() * 1000.0);
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);//函数体只有一个赋值操作：{ local_target_pt_ = local_target_pt; }
    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    // ==================== 模块 1：生成初始路径点集（STEP 1 INIT） ====================
    // 根据起点与目标点距离计算时间步长 ts（距离>0.1 用 1.5 倍，否则用 5 倍，保证控制点间距合理）
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 分支 A：首次规划 / 强制多项式 / 需要重新生成 → 用“引导段”或“多项式”得到一串路径点
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        // ---------- 1）优先尝试使用从全局路径截取的引导段 ----------
        bool use_guide = !guide_path_7m_withoutFirst5points_manager_h_.empty() && guide_path_7m_withoutFirst5points_manager_h_.size() >= 2;
        if (use_guide)
        {
          point_set.push_back(start_pt);
          for (const Eigen::Vector3d &pt : guide_path_7m_withoutFirst5points_manager_h_)
            point_set.push_back(pt);
          if ((point_set.back() - local_target_pt).norm() > 1e-3)
            point_set.back() = local_target_pt;

          // 如果引导段太短，生成的点集数量过少（例如只有起点+两段引导点=3个点），
          // 后续 B 样条拟合会失败并可能导致数值问题。此时直接退回到多项式初始化方案。
          if (point_set.size() <= 3)
          {
            std::cout << "[plan7mLocalPath]: guide segment too short ("
                      << point_set.size()
                      << " pts), fallback to polynomial init." << std::endl;
            use_guide = false;
            point_set.clear();
          }
          else
          {
            start_end_derivatives.push_back(start_vel);
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(start_acc);
            start_end_derivatives.push_back(Eigen::Vector3d::Zero());
          }
        }

        // ---------- 2）若未使用引导段（没有或太短），则退化为多项式初始化 ----------
        if (!use_guide)
        {
          // 用于存储生成的轨迹
          PolynomialPath gl_path;

          double dist = (start_pt - local_target_pt).norm();
          // 判断 速度的平方/加速度 是否大于 dist，并决定如何计算时间
          double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

          if (!flag_randomPolyTraj)
          // false 生成一段单一的多项式轨迹，true 生成一个包含随机插入点的轨迹
          {
            gl_path = PolynomialPath::one_segment_path_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
          }
          else
          {
            Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
            Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
            Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                 (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                                 (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
            Eigen::MatrixXd pos(3, 3);
            pos.col(0) = start_pt;
            pos.col(1) = random_inserted_pt;
            pos.col(2) = local_target_pt;
            Eigen::VectorXd t(2);
            t(0) = t(1) = time / 2;
            gl_path = PolynomialPath::minSnapPath(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
          }

          double t;
          bool flag_too_far;
          ts *= 1.5; // ts will be divided by 1.5 in the next
          do
          {
            ts /= 1.5;
            point_set.clear();
            flag_too_far = false;
            Eigen::Vector3d last_pt = gl_path.evaluate(0);
            for (t = 0; t < time; t += ts)
            {
              Eigen::Vector3d pt = gl_path.evaluate(t);
              if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
              {
                flag_too_far = true;
                break;
              }
              last_pt = pt;
              point_set.push_back(pt);
            }
          } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
          t -= ts;
          start_end_derivatives.push_back(gl_path.evaluateVel(0));
          start_end_derivatives.push_back(local_target_vel);
          start_end_derivatives.push_back(gl_path.evaluateAcc(0));
          start_end_derivatives.push_back(gl_path.evaluateAcc(t));
        }
      }
      // 分支 B：非首次且不强制多项式 → 从当前正在执行的轨迹上“截取从当前时刻往后”的一段，再按弧长采样得到点集
      else
      {
        double t;
        double t_cur = (rclcpp::Clock().now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_path_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_path_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialPath gl_path = PolynomialPath::one_segment_path_gen(local_data_.position_path_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_path_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_path_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_path.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              printf("\033[42m[plan7mLocalPath] 执行耗时: %.3f ms\033[0m\n", (rclcpp::Clock().now() - t_plan7m_begin).seconds() * 1000.0);
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_path_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_path_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // ==================== 模块 2：点集转 B 样条控制点 ====================
    // 用起点/终点速度加速度约束，把 point_set 拟合成一条 B 样条，得到控制点 ctrl_pts；再初始化优化器用的分段信息
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    // ==================== 模块 3：B 样条优化（避障 + 平滑，STEP 2 OPTIMIZE） ====================
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_paths;

    // 若为 true：生成多条候选路径（distinctivePaths），逐条优化后选代价最低的一条；否则只优化单条路径
    if (pp_.try_more_paths_and_choose_best)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> paths = bspline_optimizer_->distinctivePaths(segments);
      cout << "\033[1;33m"
           << "multi-paths=" << paths.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = paths.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizePathRebound(ctrl_pts_temp, final_cost, paths[i], ts))
        {

          cout << "path " << paths.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_paths.push_back(point_set);
        }
        else
        {
          cout << "path " << paths.size() - i << " failed." << endl;
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_paths, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizePathRebound(ctrl_pts, ts);
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      printf("\033[42m[plan7mLocalPath] 执行耗时: %.3f ms\033[0m\n", (rclcpp::Clock().now() - t_plan7m_begin).seconds() * 1000.0);
      return false;
    }

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    // ==================== 模块 4：时间重分配（STEP 3 REFINE） ====================
    // 若速度/加速度超限，则拉长时间轴重新参数化并再优化一次；仅单机或 drone_0 时启用
    if (pp_.dog_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refinePathAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        printf("\033[42m[plan7mLocalPath] 执行耗时: %.3f ms\033[0m\n", (rclcpp::Clock().now() - t_plan7m_begin).seconds() * 1000.0);
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = rclcpp::Clock().now() - t_start;

    // ==================== 模块 5：保存结果并返回 ====================
    updatePathInfo(pos, rclcpp::Clock().now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    printf("\033[42m[plan7mLocalPath] 执行耗时: %.3f ms\033[0m\n", (rclcpp::Clock().now() - t_plan7m_begin).seconds() * 1000.0);
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updatePathInfo(UniformBspline(control_points, 3, 1.0), rclcpp::Clock().now());

    return true;
  }

  /** 检查本机轨迹与指定机器狗 dog_id 的轨迹在时间重叠段内是否小于群控间距：若存在某时刻距离 < swarm_clearance 则返回 true（发生碰撞），否则返回 false。 */
  bool EGOPlannerManager::checkCollision(int dog_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_paths_buf_[dog_id].start_time_.toSec();
    double my_path_start_time = local_data_.start_time_.seconds();
    double other_path_start_time = swarm_paths_buf_[dog_id].start_time_.seconds();

    double t_start = max(my_path_start_time, other_path_start_time);
    double t_end = min(my_path_start_time + local_data_.duration_ * 2 / 3, other_path_start_time + swarm_paths_buf_[dog_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_path_.evaluateDeBoorT(t - my_path_start_time) - swarm_paths_buf_[dog_id].position_path_.evaluateDeBoorT(t - other_path_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalPathWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    // ---------- 模块 1：构造完整路径点序列 points = [start_pos, waypoints[0], ...] ----------
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    // ---------- 模块 2：计算路径总弧长 total_len ----------
    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // ---------- 模块 3：相邻点过远时线性插中间点，得到 inter_points（阈值 = max(total_len/8, 4.0)） ----------
    vector<Eigen::Vector3d> inter_points;
    // 距离阈值：取“路径总长/8”与 4.0m 的较大值，相邻点超过此距离则中间插点
    double dist_thresh = max(total_len / 8, 4.0);
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));  // 先加入当前段起点
      double dist = (points.at(i + 1) - points.at(i)).norm();  // 当前段长度
      if (dist > dist_thresh)
      {
        // 将当前段等分为 id_num 小段，在中间插入 (id_num - 1) 个插值点
        int id_num = floor(dist / dist_thresh) + 1;
        for (int j = 1; j < id_num; ++j)
        {
          // 线性插值：inter_pt = (1 - j/id_num)*P_i + (j/id_num)*P_{i+1}
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    inter_points.push_back(points.back());  // 补上整条路径的终点

    // ---------- 模块 4：将 inter_points 转为 3×N 矩阵 pos，并按 max_vel_ 估算每段飞行时间 time ----------
    // 插值后路径点个数，pos 为 3×pt_num，第 i 列为第 i 个点的 xyz
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];
    // 每段飞行时间：段长/最大速度；共 pt_num-1 段，time(i) 表示从第 i 点到第 i+1 点的时间
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }
    // 首段、末段时间×2，给起飞和到达终点留余量
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // ---------- 模块 5：根据点数生成全局多项式路径（≥3 点 minSnap，2 点单段），并写入 global_path_afterCalculate_ ----------
    PolynomialPath gl_path;
    if (pos.cols() >= 3)
      // 多点：最小 snap 多项式拟合，满足起止速度/加速度约束及每段时间 time
      gl_path = PolynomialPath::minSnapPath(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      // 仅起点与终点：单段多项式，从 start_pos 到 pos.col(1)，时长 time(0)
      gl_path = PolynomialPath::one_segment_path_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      // 0 或 1 个点无法生成路径
      return false;
    auto time_now = rclcpp::Clock().now();
    // 将生成的全局路径与当前时间写入 global_path_afterCalculate_，供后续局部规划/轨迹跟踪使用
    global_path_afterCalculate_.setGlobalPath(gl_path, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalPath(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialPath gl_path;
    if (pos.cols() >= 3)
      gl_path = PolynomialPath::minSnapPath(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_path = PolynomialPath::one_segment_path_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = rclcpp::Clock().now();

    global_path_afterCalculate_.setGlobalPath(gl_path, time_now);

    return true;
  }

  bool EGOPlannerManager::refinePathAlgo(UniformBspline &path, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = path.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(path, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    path = UniformBspline(ctrl_pts, 3, ts);

    double t_step = path.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < path.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(path.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizePathRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updatePathInfo(const UniformBspline &position_path, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_path_ = position_path;
    local_data_.velocity_path_ = local_data_.position_path_.getDerivative();
    local_data_.acceleration_path_ = local_data_.velocity_path_.getDerivative();
    local_data_.start_pos_ = local_data_.position_path_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_path_.getTimeSum();
    local_data_.path_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
