/**
 * @file dyn_a_star.h
 * @brief DynAStar 动态A*路径搜索算法头文件
 * 
 * 功能：在3D环境地图中进行路径搜索
 * 特点：支持动态障碍物、26方向搜索
 */

#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

// 标准库
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

// 无穷大常量
constexpr double inf = 1 >> 20;

// 前向声明
struct GridNode;
typedef GridNode *GridNodePtr;

/**
 * @brief 网格节点结构体
 * @details 表示3D地图中的一个网格单元
 */
struct GridNode
{
	/**
	 * @brief 节点状态枚举
	 * OPENSET: 在开放列表中（待探索）
	 * CLOSEDSET: 在关闭列表中（已探索）
	 * UNDEFINED: 未定义
	 */
	enum enum_state
	{
		OPENSET = 1,      // 开放集合
		CLOSEDSET = 2,    // 关闭集合
		UNDEFINED = 3      // 未定义
	};

	int rounds{0}; // 区分每次调用，用于动态规划
	
	/**
	 * @brief 当前节点状态
	 */
	enum enum_state state
	{
		UNDEFINED
	};
	
	/**
	 * @brief 节点在网格中的索引坐标 (x, y, z)
	 */
	Eigen::Vector3i index;

	/**
	 * @brief g值：从起点到当前节点的实际代价
	 */
	double gScore{inf}, 
	
	/**
	 * @brief f值：g值 + 启发式估计值
	 */
	fScore{inf};
	
	/**
	 * @brief 父节点指针，用于路径回溯
	 */
	GridNodePtr cameFrom{NULL};
};

/**
 * @brief A*节点比较器
 * @details 用于优先队列中按f值排序，f值小的优先
 */
class NodeComparator
{
public:
	/**
	 * @brief 比较函数
	 * @param node1 节点1
	 * @param node2 节点2
	 * @return node1的f值是否大于node2的f值
	 */
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

/**
 * @brief A* 路径搜索类
 * @details 使用A*算法在3D网格地图中进行路径规划
 */
class AStar
{
private:
	/**
	 * @brief 地图指针
	 */
	GridMap::Ptr grid_map_;

	/**
	 * @brief 坐标转网格索引（快速版本）
	 */
	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	/**
	 * @brief 计算对角线启发式代价
	 * @param node1 节点1
	 * @param node2 节点2
	 * @return 对角线距离
	 */
	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	
	/**
	 * @brief 计算曼哈顿启发式代价
	 */
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	
	/**
	 * @brief 计算欧几里得启发式代价
	 */
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	
	/**
	 * @brief 获取启发式值（调用上述函数）
	 */
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 坐标转索引并调整起点和终点
	 */
	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	/**
	 * @brief 网格索引转坐标
	 */
	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	
	/**
	 * @brief 坐标转网格索引
	 */
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	/**
	 * @brief 检查位置是否被占据
	 */
	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }

	/**
	 * @brief 从终点回溯到起点获取路径
	 */
	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	/**
	 * @brief 步长
	 */
	double step_size_, inv_step_size_;
	
	/**
	 * @brief 地图中心
	 */
	Eigen::Vector3d center_;
	
	/**
	 * @brief 中心索引
	 */
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	
	/**
	 * @brief 平局打破常数
	 */
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	/**
	 * @brief 搜索到的路径节点列表
	 */
	std::vector<GridNodePtr> gridPath_;

	/**
	 * @brief 3D网格节点地图
	 */
	GridNodePtr ***GridNodeMap_;
	
	/**
	 * @brief 开放集合（优先队列）
	 */
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	/**
	 * @brief 当前搜索轮次
	 */
	int rounds_{0};

public:
	/**
	 * @brief 智能指针类型
	 */
	typedef std::shared_ptr<AStar> Ptr;

