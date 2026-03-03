# ego-planner 算法流程 Q&A

## 10个核心问题

### Q1: ego-planner 的整体数据流是什么？

**答案：**
```
输入数据 → EGOReplanFSM状态机 → EGOPlannerManager 
  → GridMap/DynAStar/BsplineOptimizer → 输出BSpline轨迹 → 无人机控制器
```

输入：/odom_world (里程计)、/waypoint (目标点)、/pointcloud (障碍物点云)
输出：/planning/bspline (B样条轨迹)

---

### Q2: EGOReplanFSM 状态机有哪些状态？状态转换顺序是什么？

**答案：**
```
INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ → EMERGENCY_STOP
```

- INIT: 初始化
- WAIT_TARGET: 等待目标点
- GEN_NEW_TRAJ: 生成新轨迹
- EXEC_TRAJ: 执行轨迹
- REPLAN_TRAJ: 重规划（检测到障碍/偏离时触发）
- EMERGENCY_STOP: 紧急停止

---

### Q3: 前端路径搜索使用什么算法？如何工作？

**答案：**
使用 **DynAStar（动态A*）算法**：
- 26方向邻域搜索（三维空间）
- 对角线启发函数 `getDiagHeu()` 计算代价
- 坐标转索引 `ConvertToIndexAndAdjustStartEndPoints()`
- 回溯路径 `retrievePath()` 返回粗略路径

---

### Q4: 后端轨迹优化使用什么方法？优化目标是什么？

**答案：**
使用 **B样条（Bspline）优化**：

优化目标（代价函数）：
- λ1 平滑代价 (smoothness): 轨迹平滑
- λ2 碰撞代价 (collision): 避开障碍物
- λ3 可行性代价 (feasibility): 满足速度/加速度约束
- λ4 拟合代价 (fitness): 贴合前端路径

---

### Q5: B样条轨迹的速度是如何计算的？

**答案：**
```
v(t) = derivative(position_bspline, t)  // B样条一阶导数
a(t) = d²/dt² * position_bspline       // 二阶导数得加速度
```

约束检查：
- `|v| > max_vel_` → 缩放时间
- `|a| > max_acc_` → 增加时间
- `|j| > max_jerk_` → 平滑控制点

---

### Q6: ego-planner 如何进行碰撞检测和重规划？

**答案：**
1. **定时检查**: `checkCollisionCallback()` 每50ms检查
2. **距离判断**: `replan_thresh_` 超过阈值触发
3. **局部重规划**: `reboundReplan()` 在当前轨迹基础上重新优化
4. **速度更新**: 基于新的局部目标点重新计算速度

---

### Q7: GridMap 是如何构建3D环境地图的？

**答案：**
- 接收 `/pointcloud` 点云数据
- 使用 **Raycast（光线投射）** 算法进行占据网格更新
- 维护 `map_min_boundary_` 和 `map_max_boundary_` 地图边界
- 通过 `getInflateOccupancy()` 检测障碍物

---

### Q8: 轨迹消息 Bspline 的格式是什么？包含哪些关键字段？

**答案：**
```yaml
control_points: [[x1,y1,z1], [x2,y2,z2], ...]  # 控制点
knots: [0, 0.1, 0.2, ...]                       # 节点向量
start_time: 1234567890.123                      # 开始时间
traj_id: 0                                       # 轨迹ID
```

---

### Q9: PolynomialTraj 和 UniformBspline 有什么区别？

**答案：**
- **PolynomialTraj**: 多项式轨迹，用于前端初始轨迹生成
- **UniformBspline**: 均匀B样条曲线，用于后端优化和最终输出
- B样条的优势：局部控制性、平滑性、计算效率高

---

### Q10: ego-planner 的参数配置文件中有哪些关键参数？

**答案：**
```cpp
max_vel_          // 最大速度 (m/s)
max_acc_          // 最大加速度 (m/s²)
max_jerk_         // 最大加加速度 (m/s³)
planning_horizen_  // 规划视野距离
ctrl_pt_dist      // 控制点间距
replan_thresh_    // 重规划距离阈值
no_replan_thresh_ // 不需重规划距离阈值
```

---

## 参考资料

- 主入口: `src/planner/plan_manage/src/ego_planner_node.cpp`
- 状态机: `src/planner/plan_manage/src/ego_replan_fsm.cpp`
- 规划管理: `src/planner/plan_manage/src/planner_manager.cpp`
- B样条优化: `src/planner/bspline_opt/src/bspline_optimizer.cpp`
- A*搜索: `src/planner/path_searching/src/dyn_a_star.cpp`
