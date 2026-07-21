# SUPER — 后端轨迹优化定制

> SUPER（Safe-Ultra-fast-exploration planner）作为 USS-NAV 的 local planner 后端（super backend）时，在 `exp_traj`（MINCO-S4 轨迹优化）中加入的定制代价项与修复记录。

系统位于 `ws_main/src/planners/super_planner/`，配置入口 `bringup_test/params/super_planner/click_smooth_ros1.yaml`。

---

## 1. 优化管道与代价项全景

```
guide_path (A* on ROG-Map)
     │
     ▼
seed lines (corridor_line_max_length 上限)
     │
     ▼
CIRI convex polytopes (SFC)
     │
     ▼
MINCO-S4 exp_traj optimization (L-BFGS)
     │
     ▼
pos_cmd @100Hz (p/v/a/j + yaw)
```

`constraintsFunctional`（`exp_traj_optimizer_s4.cpp`）中的惩罚项：

| 项 | 权重配置 | 机制 |
|----|----------|------|
| corridor (pos) | `penna_pos` | 采样点必须在 SFC 半平面内（smoothedL1） |
| waypoint attractor | `penna_attract` | piece 交界点吸向 corridor 重叠区中心（死区半径） |
| vel/acc/jerk 幅值 | `penna_vel/acc/jerk` | 动力学边界（smoothedL1） |
| omg/thrust | `penna_omg/thr` | 角速度/推力边界（flatness） |
| **curve fitting** | `penna_curve_fit` | **各向异性线吸引（本文档）** |
| 时间 | `penna_t` | 总时间压力（bang-bang 倾向的来源） |

---

## 2. Curve Fitting（EGO 式局部切线吸引）

### 动机

SUPER 原始代价项中，corridor 只限界不吸引，`penna_t` 时间压力主导解的形状。长直线空旷段轨迹在 corridor 内 slalom、速度剖面 hunting → pitch 频繁摆动（实测 ±20°@~1Hz，acc 反复打满 ±4 m/s²）。

EGO-Planner 的 `curve_fitting`（`poly_traj_optimizer.cpp:1662` + `:2880`）通过把约束点拉向**初始轨迹的局部切线**解决同一问题。本节将其移植到 SUPER 的 exp_traj 优化器。

曾评估过的替代方案：由最近两个 high-level waypoint 定义倾斜平面的 slab 约束（penna_plane）——margin 内无吸引力，弯道与单一平面冲突，已废弃并移除。

### 数学定义

guide_path $P=\{p_0,\dots,p_N\}$ 按弧长 $s$ 参数化。每条轨迹 piece $i$（对应第 $i$ 个 corridor polytope）在其归一化弧长中点处取一条局部切线：

$$
a_i = P\!\left(s = S\cdot\tfrac{i+0.5}{N_{\text{piece}}}\right), \qquad
v_i = \frac{p_{k+1}-p_k}{\|p_{k+1}-p_k\|}\ \text{(所在 guide 段方向)}
$$

对优化采样点 $x$，令 $\tilde{x}=x-a_i$，$d=x^\top v_i$，各向异性度量：

$$
f = \underbrace{\frac{d^2}{a^2}}_{\text{沿切向，}a^2=100} + \underbrace{\frac{\|\tilde{x}\|^2-d^2}{b^2}}_{\text{横向，}b^2=1}
$$

代价与梯度（$w$ = `penna_curve_fit`）：

$$
J = w f^2, \qquad
\nabla_x J = 2wf\left(\frac{2d}{a^2}v_i + \frac{2}{b^2}(\tilde{x}-d\,v_i)\right)
$$

沿路径方向偏离几乎免费（权重 1/100），横向偏离全代价 —— 轨迹被"粘"在 guide path 上但不限制沿程进度。

### 与 EGO 原始实现的差异

| | EGO `curve_fitting` | SUPER 本实现 |
|---|---|---|
| 直线锚点 | 初始轨迹约束点自身 | guide_path 按弧长插值点 |
| 切向 | 相邻约束点中央差分 | 所在 guide 段方向 |
| 粒度 | 每约束点一条 | 每 piece（=每 polytope）一条 |
| 作用范围 | 轨迹前 2/3 | 全程 |
| 生成时机 | 每次优化前 `prepareFittedCurve` | 每次优化 setup 末尾 `prepareCurveFitLines` |

### 代码位置

| 功能 | 位置 |
|------|------|
| 切线预计算 | `exp_traj_optimizer_s4.cpp` `ExpTrajOpt::prepareCurveFitLines()` |
| 代价/梯度 | 同文件 `constraintsFunctional` 2.2b 节 |
| 权重装配 | 同文件构造函数 `penaltyWeights[7]` |
| 配置加载 | `traj_opt/config.hpp`（含 `penna_scale` 联动） |

### 配置

`bringup_test/params/super_planner/click_smooth_ros1.yaml`，`traj_opt/exp_traj` 下：

```yaml
penna_curve_fit: 1.0e+3    # 权重；0 = 关闭
curve_fit_a2inv: 0.01      # 沿切向度量倒数 (1/a^2)
curve_fit_b2inv: 1.0       # 横向度量倒数 (1/b^2)
```

调参指引：
- 轨迹仍横向漂移 → 增大 `penna_curve_fit`（1e3 → 1e4）
- 弯道被拉直、切弯角 → 减小 `penna_curve_fit` 或增大 `curve_fit_a2inv`（放宽沿程耦合）
- 与 corridor 冲突时 corridor 永远优先（`penna_pos=5e6` 高 3 个量级）

---

## 3. 相关修复记录

### 3.1 SFC 局部地图边界截断

`corridor_generator.cpp`：

- guide path 出 ROG 局部地图（10m 滑动窗）时，seed line 端点越出 clamp 后的 CIRI 包围盒 → `GeneratePolytopeFromLine` 必败 → `sfc_failed` 死循环（同 goal replan 95+ 次）。
- 修复：`SearchPolytopeOnPath` 遇界外 path 点截断 corridor（`hit_map_bound`）；`GeneratePolytopeFromLine/Point` 的 seed 端点在 box clamp 后同步钳入 box，保证 CIRI 输入恒合法。

### 3.2 goal 棘轮漂移（飞飘）

原 `fsm.cpp` `callReplanOnce` 每次 replan 原地执行 `getNearestInfCellNot(OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0)`：

1. 每周期在已漂移的 goal 上再吸附 → 误差单向累积（实测 goal 以 ~1.75 m/s 逃逸，飞机追出 260+ m）；
2. `start_pos`/`nearest_pt` 别名同一对象，函数内 `setConstant(NAN)` 使 `max_dis` 检查失效，单步可跳任意远；
3. 返回值被忽略，失败时 goal 残留 NaN。

修复：删除该原地吸附（A* 已有无损的 goal clamp/投影，astar.cpp `local_end_pt`）。

### 3.3 goal 先截断到 planning horizon 再搜索

原行为：A* 直接搜原始 waypoint（可能远超 `planning_horizon: 10.2`），g 预算耗尽后以 `REACH_HORIZON` 返回，路径终点是搜索前沿的任意节点，形状不可控。

修复（`super_planner.cpp` `generateExpTraj`）：waypoint 先沿 robot→goal 方向投影截断到 `temp_horizon`（占据时逐步回缩并投影到最近自由格），A* 改搜截断点 `eff_goal` → 路径终点精确落在朝向真实目标的直线上。`connected_goal` 仍对原始 goal 计算（避免截断点误触发 early-termination）。日志事件：`event=goal_truncated`。

### 3.4 A* z 方向代价惩罚（消除 z-first 平局伪影）

纯 A*（`tie_breaker≈1`）+ 欧氏代价下，所有单调路径 f 值几乎相等，堆平局弹出顺序由实现细节决定，系统性偏向"先走完 z 轴"（下楼段先垂直下降再平飞，贴楼梯边缘）。

修复（`astar.cpp`）：

- 边代价：`cost = sqrt(dx² + dy² + z_pen²·dz²)`
- EUCL 启发式：配套加权范数（正定矩阵诱导范数满足三角不等式 → 仍可采纳且一致）
- 配置：`astar/z_cost_penalty: 2.0`（1.0 = 原行为）；`astar/tie_breaker` 同步暴露为配置
- 效果：z 移动变贵 → 平局决定性偏向水平方向，下楼路径贴合坡道；guide path 更水平 → 与 curve fitting 协同更好

### 3.5 pitch 极限环参数建议

长直线 pitch ±20° 摆动的主因排序：

1. `penna_t: 12000` 时间最优压力 → bang-bang 加减速（可降至 ~3000）
2. `receding_dis: 1.0` 过短 → 每周期重排速度剖面（可升至 2.5）
3. `replan_rate: 15Hz` 轨迹拼接（stitch 只保 p/v/a 连续，jerk 阶跃）
4. `corridor_line_max_length: 0.5` → 10m 走廊 ~20 个小 polytope，边界抖动（可升至 1.5）

---

## 3.6 Waypoint Window（多航点窗口 + SUPER 内部进度判定）

上层（MissionFSM）一次向 SUPER 发布**最多 3 个航点的滑动窗口**（当前目标 + 后续 2 个），
SUPER 对每个航点重投影到最近 free voxel，链式 A* 依次通过所有可达航点，轨迹优化以
**软约束**通过重投影后的中间航点；航点 progress 判定全部在 SUPER 内部完成。

### 消息契约（`quadrotor_msgs`)

- `LocalGoalSet.msg` 扩展：`uint32 batch_id` + `float32[] waypoints`（扁平 xyz，≤3 点，
  行进序；`goal` 字段 = 窗口末点，yaw 仅作用末点）。`waypoints` 为空 → 兼容单点模式。
- 新增 `WaypointProgress.msg`（SUPER → 上层，`/drone_0_ego_planner_node/waypoint_progress`):
  `batch_id / consumed_count / active_idx / skipped_mask / all_consumed`。

### SUPER 侧管道

```
goalCallback (fsm_ros1.hpp)
  → setGoalWindow (fsm.cpp): 逐点 getNearestInfCellNot(OCCUPIED, wp, out, 3.0) 重投影
      失败点 → skipped_mask + event=wp_reproject_fail；全部失败 → 拒绝批次
  → gi_.goal_p = 当前目标 wp_list[active_idx]；lookahead 存入 planner gi_.wp_lookahead
  → generateExpTraj (super_planner.cpp): 链式 A*（逐段 horizon 预算递减 + 截断）
      到达的中间航点收集为 pass_wps
  → ExpTrajOpt::optimize(..., pass_wps): 映射到最近 corridor 交界内点 q_j
      软约束 J = penna_wp_pass * ||q_j - wp||²（热初始化 q_j = wp）
  → updateWaypointProgress (fsm.cpp, FOLLOW_TRAJ 每 tick):
      消费条件 = 距当前目标 < wp_reach_radius(0.4m)，或沿行进方向越过航点平面且横向 < 2m
      消费 → active_idx++ → 发布 WaypointProgress → 强制 new_goal replan
  → 批次末点到达 = 原 traj_finish_ + closeToGoal(0.1) → all_consumed + exec_finish_trigger
```

### MissionFSM 侧

- `pubLocalGoalWindow()`: 从 `path_inx_` 起打包 ≤3 点，单调递增 `batch_id` 发布；
  `waypointProgressCallback()`: `batch_id` 匹配时 `path_inx_ = batch_start + consumed_count`
  并重发滑动窗口（新 batch_id)。
- 删除了 `goTargetObject`/`goTargetWithWaypoint` 中基于 `dis_2_local_aim` 的航点推进调用
  （保留"末点不可达 → 强制 replan"检查与 stuck_force_advance 兜底）。
- yaw 仅在窗口末点成为当前目标后生效（中间航点 yaw 自由）。

### 配置与日志

```yaml
fsm/wp_reach_radius: 0.4          # 中间航点消费半径 [m]
traj_opt/exp_traj/penna_wp_pass: 1.0e+4   # 软过点权重（corridor=5e6 仍主导安全）
```

| 事件 | 内容 |
|------|------|
| `event=wp_batch_recv` | batch_id / size / valid / skipped_mask |
| `event=wp_reproject_fail` | 深占据跳过的航点 |
| `event=wp_consumed` | 消费 batch_id / wp_idx / orig_idx / dist |
| `event=wp_batch_done` | 批次末点到达 |
| `event=wp_pass_deviation` | 优化后交界点与航点的偏差（调权依据） |
| `event=wp_pass_map_failed` | 航点距所有 corridor 交界 >3m，约束被丢弃 |
| `event=guide_dt_nan` | 时间分配产出非法 dt（防御性守卫，正常为 0） |

### 3.6.1 后端适配与调试记录

- **后端门控**:MissionFSM 读 `fsm/enable_ego_replan`(super launch 置 false）推断后端；
  仅 SUPER 后端走窗口协议（`pubLocalGoalWindow` + `waypointProgressCallback`),
  EGO 后端保持原单点发布 + 距离推进逻辑不变。
- **`simplePMTimeAllocator` NaN 修复**（根因）：链式 A* 的段间起点是上一段 A* 终点
  （grid cell center)，下一段 A* 起点 snap 后 connection=0，导致 `dis[1] == total_dis`，
  命中 case 2/3.3 二次方程判别式理论零点，浮点误差使 delta 略负 → `sqrt(负)` = NaN
  piece time → L-BFGS 初值病态（8ms piece、vel 140+）→ 全部优化失败、飞机不动。
  修复：`geometry_utils.h` 判别式 clamp 到 ≥0；`generateExpTraj` 过滤 A* 连续重复点。
- **热初始化重叠区门控**:pass waypoint 交界点热初始化仅在 waypoint 落在该交界
  overlap polytope 内（-0.02 margin）时生效，避免不可行初值被饱和 smoothed-L1 卡住。
- **`entrypoint-test.sh` 遗留 ROS_PACKAGE_PATH 覆盖删除**：该覆盖在 bringup_test 迁移
  前写入，缺 `bringup_test` 等包导致 `$(find bringup_test)` 解析失败、roslaunch 启动即死。
  devel 空间的 setup.bash 本身提供完整路径。注意：entrypoint 烤进镜像，改动后需
  `docker compose build devel`。

---

## 4. 全链路记录（waypoint → A* → traj）

诊断链路数据分两层（符合项目 trace 规则）：

### 文本事件（ROS log → fluentbit / OpenSearch）

| 事件 | 内容 |
|------|------|
| `event=goal_truncated` | waypoint 截断：original_dis / temp_horizon / eff_goal / goal |
| `event=astar_result` | ret（REACH_GOAL/REACH_HORIZON）/ path_size / path_len / horizon / start / goal |
| `event=exp_traj_result` | duration / sfc_count / connected_goal / guide_path_len / start / end / goal |
| `event=goal_truncate_failed` | 截断点占据回缩失败 |

### bag 几何数据（viz profile）

`start_uss_nav_sim_rviz_super.sh` 默认 `TRACE_BAG_PROFILE=viz`，`docker/entrypoint.sh` 的 `VIZ_TRACE_BAG_TOPICS` 已含：

- `/super_planner_node/visualization/frontend_path`（A* guide path）
- `/super_planner_node/visualization/exp_traj`（优化轨迹）
- `/super_planner_node/visualization/exp_sfc`（安全走廊）
- `/super_planner_node/visualization/committed_traj`、`visualization/goal`

### 离线对齐分析

```bash
# 提取（容器内 rosbag → pickle）
docker run --rm --entrypoint bash -v <trace_dir>:/traces:ro -v /tmp/opencode:/work ego-planner-sim \
  -c "source /opt/ros/noetic/setup.bash && python3 /work/extract_chain.py /traces/run.bag /work/chain.pkl"
# 绘图（XY/XZ/YZ 快照 + z/v/a 时序）
uv run --script /tmp/opencode/plot_chain.py /tmp/opencode/chain.pkl /tmp/opencode/chain.png [t_snapshot]
```

---

## 5. ROG-Map 关键语义（易踩坑）

| 语义 | 位置 | 行为 |
|------|------|------|
| 界外点查询 | `prob_map.cpp:103` | `isOccupied` 返回 **false**（界外视为 free） |
| 虚拟地面/天花板 | `click_smooth_ros1.yaml` `virtual_ground_height/virtual_ceil_height` | **世界系** z 限（非 robot-centric），界外恒 occupied |
| goal 越界 | `astar.cpp:276` | 钳到地图边缘 + 2.5m 内推，投影到非占据格 |
<!-- table not formatted: invalid structure -->

J30V2 场景注意：下楼后 z 降至 -7~-10，`virtual_ground_height` 必须低于任务最低 z（当前 -12.5），否则楼下空间全部恒 occupied。
