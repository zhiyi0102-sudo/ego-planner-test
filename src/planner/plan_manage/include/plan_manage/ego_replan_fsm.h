/**
 * @file ego_replan_fsm.h
 * @brief EGOReplanFSM 状态机头文件
 * 
 * 功能：无人机轨迹规划状态机
 * 管理从初始化到轨迹执行的全过程
 * 
 * 状态机状态：
 * - INIT: 初始化
 * - WAIT_TARGET: 等待目标点
 * - GEN_NEW_TRAJ: 生成新轨迹
 * - REPLAN_TRAJ: 重规划
 * - EXEC_TRAJ: 执行轨迹
 * - EMERGENCY_STOP: 紧急停止
 */

#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

// Eigen矩阵库
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

// ROS消息类型
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

// ego-planner自定义库
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

/**
 * @brief ego-planner 状态机类
 * @details 管理无人机轨迹规划的完整流程
 * 
 * 主要功能：
 * - 状态管理：6种状态的切换
 * - 轨迹生成：调用规划器生成安全轨迹
 * - 碰撞检测：实时检测障碍物
 * - 轨迹重规划：检测到障碍时重新规划
 */
namespace ego_planner
{

  /**
   * @brief 状态机执行状态枚举
   */
  class EGOReplanFSM
  {

  private:
    /* ---------- 状态枚举 ---------- */
    
    /**
     * @brief FSM执行状态
     * INIT: 系统初始化完成，等待目标点
     * WAIT_TARGET: 已收到目标点，等待生成轨迹
     * GEN_NEW_TRAJ: 正在生成新轨迹
     * REPLAN_TRAJ: 检测到障碍，需要重规划
     * EXEC_TRAJ: 正在执行轨迹
     * EMERGENCY_STOP: 紧急停止
     */
    enum FSM_EXEC_STATE
    {
      INIT,              // 初始化状态
      WAIT_TARGET,       // 等待目标点
      GEN_NEW_TRAJ,      // 生成新轨迹
      REPLAN_TRAJ,       // 重规划轨迹
      EXEC_TRAJ,        // 执行轨迹
      EMERGENCY_STOP     // 紧急停止
    };

    /**
     * @brief 目标点类型
     * MANUAL_TARGET: 手动选择目标点
     * PRESET_TARGET: 预设目标点
     * REFERENCE_PATH: 参考路径
     */
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,    // 手动选择
      PRESET_TARGET = 2,    // 预设目标
      REFERENCE_PATH = 3     // 参考路径
    };

    /* ---------- 规划工具 ---------- */
    
    /// @brief 规划管理器，负责生成和优化轨迹
    EGOPlannerManager::Ptr planner_manager_;
    
    /// @brief 可视化工具，用于在RViz中显示
    PlanningVisualization::Ptr visualization_;
    
    /// @brief 数据显示消息
    ego_planner::DataDisp data_disp_;

    /* ---------- 参数 ---------- */
    
    /// @brief 目标点类型 (1=手动选择, 2=预设)
    int target_type_; 
    
    /// @brief 不需要重规划的阈值距离
    double no_replan_thresh_, 
    /// @brief 需要重规划的阈值距离
           replan_thresh_;
    
    /// @brief 预设航点数组 [50][3] (x, y, z坐标)
    double waypoints_[50][3];
    
    /// @brief 预设航点数量
    int waypoint_num_;
    
    /// @brief 规划视野范围
    double planning_horizen_, 
    /// @brief 规划时间视野
           planning_horizen_time_;
    
    /// @brief 紧急停止时间
    double emergency_time_;

    /* ---------- 规划数据 ---------- */
    
    /// @brief 是否触发生成轨迹
    bool trigger_, 
    /// @brief 是否有目标点
           have_target_, 
    /// @brief 是否有里程计数据
           have_odom_, 
    /// @brief是否有新目标点
           have_new_target_;
    
    /// @brief 当前执行状态
    FSM_EXEC_STATE exec_state_;
    
    /// @brief 状态连续执行次数
    int continously_called_times_{0};

    /// @brief 里程计位置 (x, y, z)
    Eigen::Vector3d odom_pos_, 
    /// @brief 里程计速度 (vx, vy, vz)
                   odom_vel_, 
    /// @brief 里程计加速度 (ax, ay, az)
                   odom_acc_; 
    
    /// @brief 里程计姿态 (四元数)
    Eigen::Quaterniond odom_orient_;

    /// @brief 起点状态
    Eigen::Vector3d init_pt_, 
    /// @brief 起点位置
                   start_pt_, 
    /// @brief 起点速度
                   start_vel_, 
    /// @brief 起点加速度
                   start_acc_, 
    /// @brief 起点偏航角
                   start_yaw_; 
    
    /// @brief 终点位置
    Eigen::Vector3d end_pt_, 
    /// @brief 终点速度
                   end_vel_; 
    
    /// @brief 局部目标点位置
    Eigen::Vector3d local_target_pt_, 
    /// @brief 局部目标点速度
                           local_target_vel_; 
    
    /// @brief 当前航点索引
    int current_wp_;

    /// @brief 是否触发紧急逃离
    bool flag_escape_emergency_;

    /* ---------- ROS工具 ---------- */
    
    /// @brief ROS节点句柄
    ros::NodeHandle node_;
    
    /// @brief 执行定时器 (10ms周期)
    ros::Timer exec_timer_, 
    /// @brief 安全检查定时器 (50ms周期)
             safety_timer_;
    
    /// @brief 订阅者：目标点、里程计
    ros::Subscriber waypoint_sub_, odom_sub_;
    
    /// @brief 发布者：轨迹、数据显示
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

