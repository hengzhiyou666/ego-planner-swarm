// rcl = ROS Client Library（ROS 客户端库），rclcpp = 其 C++ 实现；提供节点、spin、init/shutdown 等
#include <rclcpp/rclcpp.hpp>  

#include <visualization_msgs/msg/marker.hpp>  // 可视化消息
#include <iostream>  // 标准输入输出流

#include <ego_planner/ego_replan_fsm.h>  // EGO 规划器状态机：等待目标、规划、执行、重规划等状态控制

using namespace ego_planner;  // 使用 ego_planner 命名空间

int main(int argc, char **argv)  // 主函数，程序入口
{
  rclcpp::init(argc, argv);  // 初始化 ROS 客户端库

  // std::make_shared<T>(...)：在堆上创建对象并返回 std::shared_ptr，由智能指针管理内存、自动释放
  // rclcpp::Node：ROS2 节点类型。合起来即：用智能指针创建一个名为 "ego_planner_node" 的 ROS2 节点
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  // 在栈上创建 EGO 规划器状态机（负责状态切换与规划调度），init 绑定节点并完成初始化
  classEGOPlannerStateMachine egoPlanner_stateMachine;
  egoPlanner_stateMachine.init(node);

  rclcpp::spin(node);   // 阻塞式运行节点：持续处理回调（定时器、订阅等），直到 Ctrl+C 或 shutdown
  rclcpp::shutdown();   // 关闭 ROS 客户端库，释放资源

  return 0;
}