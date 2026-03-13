// rcl = ROS Client Library（ROS 客户端库），rclcpp = 其 C++ 实现；提供节点、spin、init/shutdown 等
#include <rclcpp/rclcpp.hpp>  

#include <visualization_msgs/msg/marker.hpp>  // 可视化消息
#include <iostream>  // 标准输入输出流

#include <ego_planner/ego_replan_fsm.h>  // 包含 EGO 重规划有限状态机（FSM）的定义

using namespace ego_planner;  // 使用 ego_planner 命名空间

int main(int argc, char **argv)  // 主函数，程序入口
{
  rclcpp::init(argc, argv);  // 初始化 ROS 客户端库

  // std::make_shared<T>(...)：在堆上创建对象并返回 std::shared_ptr，由智能指针管理内存、自动释放
  // rclcpp::Node：ROS2 节点类型。合起来即：用智能指针创建一个名为 "ego_planner_node" 的 ROS2 节点
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  // 在栈上创建一个 EGO 重规划状态机对象 rebo_replan，下一行 init(node) 会用它绑定节点并完成初始化
  EGOReplanFSM rebo_replan;

  rebo_replan.init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}