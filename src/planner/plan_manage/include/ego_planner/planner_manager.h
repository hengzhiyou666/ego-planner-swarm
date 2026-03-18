#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>      // B 样条优化器：负责把一条粗路径优化成平滑、安全的 B 样条轨迹
#include <bspline_opt/uniform_bspline.h>        // 均匀 B 样条类：表示轨迹本身，可按时间求位置/速度/加速度
#include <path_tools/msg/data_disp.hpp>         // 可视化/调试用的数据消息（路径、代价等的辅助信息）
#include <plan_env/grid_map.h>                  // 栅格地图环境：碰撞检测、占据情况、膨胀障碍物等
#include <plan_env/obj_predictor.h>             // 动态目标/其他无人机的轨迹预测模块
#include <path_tools/plan_container.hpp>        // 规划中用到的各种路径数据结构（全局路径、本地路径、编队路径等）
#include <rclcpp/rclcpp.hpp>                    // ROS2 C++ 客户端库：节点、日志、计时器等
#include <path_tools/planning_visualization.h>  // 规划结果可视化：在 RViz 里画路径、控制点、参考点等

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    // 给当前起点/终点做一次“弹跳式重规划”：在当前路径基础上重新生成一条更安全/更顺滑的局部路径
    bool plan7mLocalPath(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // 紧急停车：在当前位置附近快速生成一条“刹停路径”，让无人机尽快、安全地停下来
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 用“起点 + 终点”的方式规划一条全局路径（不带中间路点）
    bool planGlobalPath(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 用“起点 + 一串中间路点 + 终点”的方式规划一条全局路径
    bool planGlobalPathWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // 初始化规划模块：绑定 ROS2 节点、可视化对象等（在 FSM 的 init 里调用一次）
    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    // 把当前缓存的编队路径指针交给优化器，用于多机避碰
    void deliverPathToOptimizer(void) { bspline_optimizer_->setSwarmPaths(&swarm_paths_buf_); };

    // 告诉优化器“我是谁”：设置当前机器狗的 ID，方便做编队避障
    void setDogIdtoOpt(void) { bspline_optimizer_->setDogId(pp_.dog_id); }

    // 读取“编队安全间距”这个参数（两机之间至少要相距多少米）
    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    // 检查与指定 ID 的其他机器狗路径是否发生碰撞
    bool checkCollision(int dog_id);
    

    PlanParameters pp_;
    LocalPathData local_data_;
    GlobalPathData global_path_afterCalculate_;
    GridMap::Ptr grid_map_;
    /* 局部规划使用的引导路径：从全局路径上最近点起向前约 7m 的一段 */
    std::vector<Eigen::Vector3d> guide_path_7m_withoutFirst5points_manager_h_;
    void copy7mPath(const std::vector<Eigen::Vector3d> &seg) { guide_path_7m_withoutFirst5points_manager_h_ = seg; }
    const std::vector<Eigen::Vector3d> &giveMe7mPathPoints() const { return guide_path_7m_withoutFirst5points_manager_h_; }
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    SwarmPathData swarm_paths_buf_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};

    void updatePathInfo(const UniformBspline &position_path, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refinePathAlgo(UniformBspline &path, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif