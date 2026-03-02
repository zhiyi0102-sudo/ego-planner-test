# ego-planner 代码架构分析

## 1. 项目概述

ego-planner 是一个**无人机实时轨迹规划系统**，基于 ROS (Robot Operating System) 开发。主要功能是使无人机能够在复杂环境中进行自主避障飞行规划。

## 2. 核心模块

```
┌─────────────────────────────────────────────────────────────────┐
│                     ego_planner_node (主入口)                    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    EGOReplanFSM (状态机)                         │
│  状态: INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ          │
│        → REPLAN_TRAJ → EMERGENCY_STOP                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  EGOPlannerManager (规划管理器)                   │
│  - reboundReplan()    - 局部重规划                              │
│  - EmergencyStop()    - 紧急停止                                │
│  - planGlobalTraj()   - 全局轨迹规划                            │
└─────────────────────────────────────────────────────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
   ┌────────────┐      ┌────────────┐      ┌────────────┐
   │ bspline_opt│      │ path_search│      │  plan_env │
   │ (B样条优化)│      │ (A*搜索)   │      │ (网格地图) │
   └────────────┘      └────────────┘      └────────────┘
```

## 3. 主程序接口

### 3.1 节点入口
- **文件**: `src/planner/plan_manage/src/ego_planner_node.cpp`
- **功能**: 初始化 ROS 节点，创建 EGOReplanFSM 实例

```cpp
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);
  ros::spin();
}
```

### 3.2 核心类接口

#### EGOReplanFSM (状态机)
- **init(nh)**: 初始化
- **callReboundReplan()**: 调用重规划
- **callEmergencyStop()**: 紧急停止

#### EGOPlannerManager (规划管理器)
- **reboundReplan()**: 局部重规划
- **EmergencyStop()**: 紧急停止
- **planGlobalTraj()**: 全局轨迹规划
- **initPlanModules()**: 初始化规划模块

## 4. 模块详解

### 4.1 plan_env (环境感知)
- **GridMap**: 3D 网格地图构建
- **Raycast**: 光线投射障碍物检测
- 功能：环境建模、碰撞检测

### 4.2 path_searching (路径搜索)
- **DynAStar**: 动态 A* 算法
- 功能：前端路径搜索

### 4.3 bspline_opt (轨迹优化)
- **BsplineOptimizer**: B样条轨迹优化
- **UniformBspline**: 均匀 B 样条曲线
- 功能：后端轨迹优化、时间参数化

### 4.4 traj_utils (可视化)
- **PlanningVisualization**: RViz 可视化
- 功能：轨迹可视化、调试

## 5. 知识网络图

```
[用户/外部指令]
       │
       ▼
[waypoint_sub] ──► [EGOReplanFSM 状态机]
                            │
            ┌───────────────┼───────────────┐
            │               │               │
            ▼               ▼               ▼
     [GridMap]      [DynAStar]    [BsplineOptimizer]
     (环境)          (前端)           (后端)
            │               │               │
            └───────────────┼───────────────┘
                            │
                            ▼
                   [EGOPlannerManager]
                            │
                            ▼
                   [轨迹发布 bspline_pub]
                            │
                            ▼
                   [无人机控制器]
```

## 6. 关键消息

- **输入**: 
  - `/odom` - 无人机里程计
  - `/waypoint` - 目标点
  - `/imu` - IMU 数据

- **输出**:
  - `/bspline` - B样条轨迹
  - `/visualization` - 可视化数据

## 7. 依赖

- ROS (Kinetic/Melodic)
- Eigen (线性代数)
- PCL (点云处理)
- OpenCV (图像处理)

## 8. 编译与运行

```bash
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```
