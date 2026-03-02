# ego-planner 数据流与速度规划详解

## 1. 数据流动总览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           外部输入数据                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│  /odom_world          无人机里程计 (位置、速度、姿态)                        │
│  /waypoint_generator/waypoints  目标点 (用户指定/预设)                       │
│  /imu                 IMU数据 (加速度、角速度)                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     EGOReplanFSM 状态机                                      │
│  INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      EGOPlannerManager 规划管理器                              │
└─────────────────────────────────────────────────────────────────────────────┘
         │                         │                         │
         ▼                         ▼                         ▼
┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
│   GridMap       │   │   DynAStar      │   │ BsplineOptimizer│
│  (环境感知)     │   │  (前端搜索)      │   │  (后端优化)      │
│                 │   │                 │   │                 │
│ 点云 → 占据网格  │   │ 起点→终点搜索  │   │  控制点→B样条  │
│ 碰撞检测        │   │  初步路径      │   │  速度/加速度限制│
└─────────────────┘   └─────────────────┘   └─────────────────┘
         │                         │                         │
         └─────────────────────────┼─────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        输出: /planning/bspline                               │
│  包含: 控制点、时间间隔、轨迹ID                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          无人机控制器                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 2. 路径生成流程 (Step by Step)

### Step 1: 接收目标点
```
waypointCallback() / planGlobalTrajbyGivenWps()
  │
  ├─ 读取目标点位置 (end_pt_)
  ├─ 读取当前位置 (odom_pos_)
  └─ 调用 planner_manager_->planGlobalTraj()
```

### Step 2: 全局路径规划
```
planGlobalTraj() / planGlobalTrajWaypoints()
  │
  ├─ 使用 PolynomialTraj 生成初始轨迹
  │   time = dist / max_vel (考虑加速和减速)
  │
  ├─ 调用 bspline_optimizer_->optimize()
  │   │
  │   ├─ 调用 a_star_->search()  (前端A*搜索)
  │   │
  │   └─ 调用 refineTraj()       (后端B样条优化)
  │
  └─ 输出: UniformBspline 轨迹
```

### Step 3: 前端 A* 搜索 (DynAStar)
```
a_star_->search()
  │
  ├─ getDiagHeu()      计算对角线启发函数
  ├─ ConvertToIndexAndAdjustStartEndPoints() 坐标转索引
  ├─ 26方向邻域搜索
  ├─ retrievePath()     回溯路径
  └─ 返回: vector<GridNodePtr> 粗略路径
```

### Step 4: 后端 B样条优化 (BsplineOptimizer)
```
optimize() / refineTrajAlgo()
  │
  ├─ initControlPoints()    初始化控制点
  │   └─ 分割受障碍物影响的轨迹段
  │
  ├─ computeCost()          计算代价函数
  │   ├─ lambda1: 平滑代价 (smoothness)
  │   ├─ lambda2: 碰撞代价 (collision)
  │   ├─ lambda3: 可行性代价 (feasibility)
  │   └─ lambda4: 拟合代价 (fitness)
  │
  ├─ optimizeB spline()    梯度下降优化
  │
  └─ 输出: 优化后的 UniformBspline 轨迹
```

## 3. 速度规划详解

### 3.1 速度约束
```cpp
// 参数配置文件中的关键参数
max_vel_    // 最大速度 (m/s)
max_acc_    // 最大加速度 (m/s²)
max_jerk_   // 最大加加速度 (m/s³)
```

### 3.2 速度计算公式
```
时间计算:
  t = dist / v_max                    // 匀速
  t = sqrt(2 * dist / a_max)          // 加速-减速
  
速度 v(t) = derivative(position_bspline, t)  // B样条求导

约束检查:
  if (|v| > max_vel_)   缩放时间
  if (|a| > max_acc_)   增加时间
  if (|j| > max_jerk_)  平滑控制点
```

### 3.3 B样条速度特性
```
UniformBspline 特点:
  - 位置: P(t) = Σ B_i(t) * P_i      (B_i 是B样条基函数)
  - 速度: P'(t) = d/dt * P(t)        (对控制点求导)
  - 加速度: P''(t) = d²/dt² * P(t)   (二阶导数)
```

## 4. 关键数据结构

### 4.1 轨迹数据
```cpp
struct LocalTrajData {
  int traj_id_;                    // 轨迹ID
  UniformBspline position_traj_;  // 位置B样条
  double start_time_;              // 开始时间
  ros::Time  start_time_ws_;      // ROS时间
};

struct GlobalTrajData {
  PolynomialTraj global_traj_;     // 多项式全局轨迹
  double global_duration_;         // 全局时长
};
```

### 4.2 规划参数
```cpp
struct PlanParameters {
  double max_vel_;        // 最大速度
  double max_acc_;        // 最大加速度
  double max_jerk_;       // 最大加加速度
  double planning_horizen_; // 规划视野距离
  double ctrl_pt_dist;    // 控制点间距
};
```

## 5. 消息流程

### 5.1 输入消息
| Topic | 类型 | 作用 |
|-------|------|------|
| /odom_world | nav_msgs/Odometry | 无人机当前位置速度 |
| /waypoint_generator/waypoints | nav_msgs/Path | 目标点 |
| /pointcloud | sensor_msgs/PointCloud2 | 障碍物点云 |

### 5.2 输出消息
| Topic | 类型 | 作用 |
|-------|------|------|
| /planning/bspline | ego_planner/Bspline | 优化后的B样条轨迹 |
| /planning/data_display | ego_planner/DataDisp | 可视化数据 |

### 5.3 Bspline 消息格式
```yaml
# ego_planner/Bspline
control_points: [[x1,y1,z1], [x2,y2,z2], ...]  # 控制点
knots: [0, 0.1, 0.2, ...]                       # 节点向量
start_time: 1234567890.123                      # 开始时间
traj_id: 0                                       # 轨迹ID
```

## 6. 重规划机制

当无人机飞行过程中:
1. **定时检查**: `checkCollisionCallback()` 每50ms检查碰撞
2. **距离判断**: `replan_thresh_` 超过阈值触发重规划
3. **局部重规划**: `reboundReplan()` 在当前轨迹基础上重新优化
4. **速度更新**: 基于新的局部目标点重新计算速度曲线

```
正常飞行 → 检测到障碍/偏离 → 触发REPLAN_TRAJ 
         → 调用 reboundReplan() → 重新优化B样条 → 发布新轨迹
```
