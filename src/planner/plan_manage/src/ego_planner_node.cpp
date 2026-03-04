/**
 * @file ego_planner_node.cpp
 * @brief ego-planner 主入口节点
 * 
 * 功能：初始化ROS节点，创建并启动EGOReplanFSM状态机
 * 
 * 作者：ego-planner团队
 * 许可证：MIT
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

/**
 * @brief 主函数 - 程序入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 * 
 * 工作流程：
 * 1. 初始化ROS节点，节点名为"ego_planner_node"
 * 2. 创建私有节点句柄("~")
 * 3. 创建EGOReplanFSM状态机实例
 * 4. 初始化状态机
 * 5. 等待1秒确保初始化完成
 * 6. 进入ROS主循环，等待回调
 */
int main(int argc, char **argv)
{
  // 步骤1: 初始化ROS客户端库
  // 参数：argc, argv, 节点名称
  ros::init(argc, argv, "ego_planner_node");
  
  // 步骤2: 创建节点句柄
  // "~"表示私有命名空间，可以访问节点私有参数
  ros::NodeHandle nh("~");
  
  // 步骤3: 创建状态机实例
  // EGOReplanFSM是核心状态机类，负责管理整个规划流程
  EGOReplanFSM rebo_replan;
  
  // 步骤4: 初始化状态机
  // 读取参数、订阅话题、发布话题、启动定时器
  rebo_replan.init(nh);
  
  // 步骤5: 等待1秒
  // 让状态机有足够时间完成初始化
  ros::Duration(1.0).sleep();
  
  // 步骤6: 进入ROS主循环
  // 处理所有回调函数(定时器、订阅消息等)
  ros::spin();
  
  // 程序正常退出
  return 0;
}
