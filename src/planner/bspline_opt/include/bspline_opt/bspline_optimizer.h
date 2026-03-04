/**
 * @file bspline_optimizer.h
 * @brief BsplineOptimizer B样条轨迹优化器头文件
 * 
 * 功能：使用B样条和梯度下降算法优化无人机轨迹
 * 目标：生成平滑、安全、动力学可行的轨迹
 */

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

// Eigen矩阵库
#include <Eigen/Eigen>

// ego-planner库
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elastic band optimization
// 使用梯度下降和弹性带优化

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

  /**
   * @brief 控制点类
   * @details 存储B样条轨迹的控制点和相关信息
   */
  class ControlPoints
  {
  public:
    double clearance;  // 离障碍物的距离/间隙
    
    int size;          // 控制点数量
    
    Eigen::MatrixXd points;  // 控制点坐标 (3 x N 矩阵)
    
    /**
     * @brief 基础点向量（障碍物碰撞点）
     * @details 每个控制点对应的基础点，用于碰撞检测
     */
    std::vector<std::vector<Eigen::Vector3d>> base_point; 
    
    /**
     * @brief 方向向量
     * @details 必须归一化的方向向量
     */
    std::vector<std::vector<Eigen::Vector3d>> direction;  
    
    /**
     * @brief 临时标志位
     * @details 在多个地方使用的标志位，每次使用前需要初始化
     */
    std::vector<bool> flag_temp;                          

    /**
     * @brief 调整控制点数组大小
     * @param size_set 新的控制点数量
     */
    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
    }
  };

  /**
   * @brief B样条轨迹优化器
   * 
   * 功能：
   * - 接收初始轨迹（控制点）
   * - 使用梯度下降算法优化
   * - 考虑多个代价函数（平滑、碰撞、可行性）
   * - 输出优化后的B样条轨迹
   */
  class BsplineOptimizer
  {

  public:
    /**
     * @brief 构造函数
     */
    BsplineOptimizer() {}
    
    /**
     * @brief 析构函数
     */
    ~BsplineOptimizer() {}

    /* 主要API */

    /**
     * @brief 设置环境地图
     * @param env 智能指针，指向GridMap实例
     */
    void setEnvironment(const GridMap::Ptr &env);
    
    /**
     * @brief 设置优化参数
     * @param nh ROS节点句柄
     * 
     * 读取的参数：
     * - lambda_smooth: 平滑权重
     * - lambda_collision: 碰撞权重
     * - lambda_feasibility: 可行性权重
     * - lambda_fitness: 拟合权重
     * - max_vel: 最大速度
     * - max_acc: 最大加速度
     */
    void setParam(ros::NodeHandle &nh);
    
    /**
     * @brief B样条轨迹优化主函数
     * @param points 初始控制点 (3 x N 矩阵)
     * @param ts 时间步长
     * @param cost_function 代价函数类型
     * @param max_num_id 最大迭代次数ID
     * @param max_time_id 最大时间ID
     * @return 优化后的控制点矩阵
     */
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* 辅助函数 */

