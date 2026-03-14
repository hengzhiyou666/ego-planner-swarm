#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "path_tools/msg/bspline.hpp"
#include "path_tools/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_tools/msg/data_disp.hpp"
#include "ego_planner/planner_manager.h"
#include "path_tools/planning_visualization.h"

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    // 规划状态机执行状态：从初始化 → 等目标 → 生成/重规划轨迹 → 执行 → 必要时紧急停
    enum FSM_EXEC_STATE
    {
      INIT,             // 初始化
      WAIT_TARGET,      // 等待目标（尚未收到目标点或参考路径）
      GEN_NEW_PATH,     // 生成新路径（首次全局规划）
      REPLAN_PATH,      // 重规划路径（飞行中局部/全局重规划）
      EXEC_PATH,        // 执行路径（按当前路径飞行）
      EMERGENCY_STOP,   // 紧急停止
      SEQUENTIAL_START  // 顺序启动（多机/编队等场景）
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      USE_GLOBAL_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    path_tools::msg::DataDisp data_disp_;
    path_tools::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[200][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool plan_xy_only_{false};  // 机器狗等：仅用 xy，z 强制为 0

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    bool have_pct_path_{false};
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    /** 从 /pct_path_unfinished 取前 7m 的点（按顺序），去掉最前 5 个点后送给 EGO Planner 的引导段 */
    std::vector<Eigen::Vector3d> pct_guide_segment_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<path_tools::msg::MultiBsplines>::SharedPtr swarm_paths_sub_;
    rclcpp::Subscription<path_tools::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pct_path_sub_;   // 外部给的一整条参考路径（/pct_path）

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<path_tools::msg::Bspline>::SharedPtr bspline_pub_;          // 当前本机优化后的 B 样条路径
    rclcpp::Publisher<path_tools::msg::DataDisp>::SharedPtr data_disp_pub_;       // 调试/可视化用数据
    rclcpp::Publisher<path_tools::msg::MultiBsplines>::SharedPtr swarm_paths_pub_; // 发送本机路径给其他无人机
    rclcpp::Publisher<path_tools::msg::Bspline>::SharedPtr broadcast_bspline_pub_; // 向所有无人机广播路径
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pct_path_unfinished_pub_;    // 发布“从当前位置开始还没走完的参考路径”/pct_path_unfinished

    /* helper functions */
    bool callPlanLocalPath(bool flag_use_poly_init, bool flag_randomPolyTraj); // 调用规划局部路径
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalPath(const int trial_times = 1);
    bool planFromCurrentPath(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void pctPathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg);
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmPathsCallback(const std::shared_ptr<const path_tools::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const path_tools::msg::Bspline> &msg);

    bool checkCollision();
    void publishSwarmPaths(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(rclcpp::Node::SharedPtr &node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif