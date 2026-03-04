/**
 * @file planner_manager.h
 * @brief EGOPlannerManager 轨迹规划管理器头文件
 * 
 * 功能：无人机轨迹规划的核心管理类
 * 负责协调前端路径搜索和后端轨迹优化
 */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

// 标准库
#include <stdlib.h>

// ego-planner自定义库
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  /**
   * @brief EGOPlannerManager 轨迹规划管理器
   * 
   * 功能：
   * - 初始化各个规划模块
   * - 接收起点和终点，生成安全平滑的轨迹
   * - 处理轨迹重规划
   * - 紧急停止
   * 
   * 核心算法：
   * - 前端：A* 路径搜索 (DynAStar)
   * - 后端：B样条轨迹优化 (BsplineOptimizer)
   */
  class EGOPlannerManager
  {
    // SECTION stable 稳定API
  public:
    /**
     * @brief 构造函数
     */
    EGOPlannerManager();
    
    /**
     * @brief 析构函数
     */
    ~EGOPlannerManager();

    // Eigen矩阵对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* 主要规划接口 */

    /**
     * @brief 局部重规划
     * @param start_pt 起点位置
     * @param start_vel 起点速度
     * @param start_acc 起点加速度
     * @param end_pt 终点位置
     * @param end_vel 终点速度
     * @param flag_polyInit 是否使用多项式初始化
     * @param flag_randomPolyTraj 是否使用随机多项式轨迹
     * @return 是否规划成功
     * 
     * 当检测到障碍物或偏离轨迹时调用此函数进行局部重规划
     */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    
    /**
     * @brief 紧急停止
     * @param stop_pos 停止位置
     * @return 是否成功停止
     * 
     * 当检测到碰撞风险时，立即生成停止轨迹
     */
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    
    /**
     * @brief 全局轨迹规划（单目标点）
     * @param start_pos 起点位置
     * @param start_vel 起点速度
     * @param start_acc 起点加速度
     * @param end_pos 终点位置
     * @param end_vel 终点速度
     * @param end_acc 终点加速度
     * @return 是否规划成功
     */
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    
    /**
     * @brief 全局轨迹规划（多目标点/航点）
     * @param start_pos 起点位置
     * @param start_vel 起点速度
     * @param start_acc 起点加速度
     * @param waypoints 航点列表
     * @param end_vel 终点速度
     * @param end_acc 终点加速度
     * @return 是否规划成功
     */
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /**
     * @brief 初始化规划模块
     * @param nh ROS节点句柄
     * @param vis 可视化工具指针
     * 
     * 初始化内容：
     * - 读取参数
     * - 初始化GridMap环境地图
     * - 初始化BsplineOptimizer优化器
     * - 初始化A*搜索
     */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    // 规划数据
    PlanParameters pp_;           // 规划参数
    LocalTrajData local_data_;   // 局部轨迹数据
    GlobalTrajData global_data_; // 全局轨迹数据
    GridMap::Ptr grid_map_;      // 环境地图指针

  private:
    // 规划算法和模块
    PlanningVisualization::Ptr visualization_;  // 可视化工具
    
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;  // B样条优化器（用于重规划）

    int continous_failures_count_{0};  // 连续失败计数

    /**
     * @brief 更新轨迹信息
     * @param position_traj 位置轨迹
     * @param time_now 当前时间
     */
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    /**
     * @brief 重参数化B样条
     * @param bspline 原始B样条
     * @param start_end_derivative 起点和终点的导数
     * @param ratio 比例
     * @param ctrl_pts 控制点输出
     * @param dt 时间步长输出
     * @param time_inc 时间增量输出
     */
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    /**
     * @brief 精炼轨迹算法
     * @param traj 输入轨迹
     * @param start_end_derivative 起点和终点导数
     * @param ratio 比例
     * @param ts 时间步长
     * @param optimal_control_points 最优控制点输出
     * @return 是否成功
     */
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing 开发中的功能
  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
