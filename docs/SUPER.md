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

### 3.3 pitch 极限环参数建议

长直线 pitch ±20° 摆动的主因排序：

1. `penna_t: 12000` 时间最优压力 → bang-bang 加减速（可降至 ~3000）
2. `receding_dis: 1.0` 过短 → 每周期重排速度剖面（可升至 2.5）
3. `replan_rate: 15Hz` 轨迹拼接（stitch 只保 p/v/a 连续，jerk 阶跃）
4. `corridor_line_max_length: 0.5` → 10m 走廊 ~20 个小 polytope，边界抖动（可升至 1.5）

---

## 4. ROG-Map 关键语义（易踩坑）

| 语义 | 位置 | 行为 |
|------|------|------|
| 界外点查询 | `prob_map.cpp:103` | `isOccupied` 返回 **false**（界外视为 free） |
| 虚拟地面/天花板 | `click_smooth_ros1.yaml` `virtual_ground_height/virtual_ceil_height` | **世界系** z 限（非 robot-centric），界外恒 occupied |
| goal 越界 | `astar.cpp:276` | 钳到地图边缘 + 2.5m 内推，投影到非占据格 |
<!-- table not formatted: invalid structure -->

J30V2 场景注意：下楼后 z 降至 -7~-10，`virtual_ground_height` 必须低于任务最低 z（当前 -12.5），否则楼下空间全部恒 occupied。
