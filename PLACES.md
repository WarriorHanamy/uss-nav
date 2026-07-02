# Places → EGO Goal 调用链

> places（Polyhedron / 多面体）是拓扑骨架的节点。目标导航和前沿探索均通过在多面体邻接图上 A* 搜索，最终转化为 EgoGoalSet 下发至 EGO Planner。

---

## 概述

places 的本质是凸多面体（QuickHull 产物），每个 place 维护一个邻接边表 `edges_`，形成一张无向图。所有路径搜索在这张图上完成。

```
places 邻接图 (Polyhedron.edges_)
        │
        ▼ astarSearch()
多面体中心点序列 path_res_
        │
        ▼ getAndPublishNextAim()
pubLocalGoal(aim_pos, yaw, ...)
        │
        ▼ ego_goal_pub_.publish()
EgoGoalSet → /local_goal → EGOReplanFSM
```

---

## 涉及文件（5 层）

```
skeleton_generation.cpp    骨架生成 + 邻接图构建
        │
        ▼
skeleton_astar.cpp         A* 搜索
        │
        ▼
scene_graph.cpp            物体→骨架桥接 + 调用 astarSearch
frontier_finder.cpp        TSP 前沿代价矩阵 + 调用 astarSearch
        │
        ▼
fast_exploration_fsm.cpp   FSM 编排 + pubLocalGoal → EgoGoalSet
```

---

## 1. 骨架生成 (skeleton_generation.cpp)

places 的生成和连接入口在 `expandSkeleton()`，核心逻辑：

### place 创建
- `generatePolyVertices` — 射线投射 → QuickHull → 凸多面体
- 初始化 `edges_`, `connected_nodes_`, 分配 `area_id_`

### 邻接图构建
```
扩展时 (processAValidFrontier, :828-878):
master_poly ↔ gate_poly ↔ new_poly    (connected_nodes_)
master_poly ←——→ new_poly              (edges_, A* 直接使用的边)

回环时 (findLoopbackConnectionFromCandidate, :933-946):
new_poly 与已有的 raycast 命中多面体建立边

强制连接 (findNewTopoConnection, :880-931):
若无合法邻居，fallback 连接到 last_mount_polyhedron_
```

- `connected_nodes_` 包含 gate 节点，`edges_` 跳过 gate，A* 只走 `edges_`
- 边是双向的，添加时间时写入两边

---

## 2. A* 搜索 (skeleton_astar.cpp)

**入口**: `SkeletonAstar::astarSearch(PolyHedronPtr start, PolyHedronPtr end)` (:29)

```
邻居展开 (getNeighborPolyhedronsNotInCloseList, :17-27):
for (const auto& edge : cur_node->polyhedron_->edges_)
    neighbor = edge.poly_nxt_      // edges_ 中的每条边指向一个邻居 place

启发式: Euclidean distance between polyhedron centers
路径: 多面体中心点序列 (path_)
```

调用方式（两种重载，skeleton_generation.cpp:144/167）：
- `astarSearch(start_poly, end_poly, path)` — 以 place 为节点
- `astarSearch(start_point, end_point, path, ...)` — 先 mount 到最近 place 再搜

---

## 3. 物体→骨架桥接 (scene_graph.cpp)

物体通过 `ObjectNode.edge.polyhedron_father` 挂载到某个 place。

**物体导航入口** — `SceneGraph::getPathToObjectWithId()` (:88-119)：

```cpp
obj = object_map_[id];
// 检查 cur_poly_ 和 obj->edge.polyhedron_father
dis = skeleton_gen_->astarSearch(cur_poly_, obj->edge.polyhedron_father, path);
aim_pos = obj->edge.polyhedron_father->center_;
aim_yaw = atan2(obj父中心 - obj位置) + π;    // 面向物体
```

> 物体没有 `polyhedron_father`（nullptr）时直接返回 false，不会搜路。

---

## 4. TSP 前沿代价 (frontier_finder.cpp)

探索模式中，frontier 需要与 place 图关联以计算代价矩阵。

**核心函数** — `FrontierFinder::getPathWithTopo()` (:755-775)：

```cpp
// 短距可见: 直连线段 path
if (distance < 2*sensor_range && isVisible)
    path = {start_pose, end_pose};
// 否则走骨架图 A*
else
    dis = scene_graph_->skeleton_gen_->astarSearch(start_poly, end_poly, path);
```

调用位置:
- TSP 代价矩阵构建 (:584, 692, 1389)
- 路径恢复与重规划 (:751, 771, 1262)

---

## 5. FSM → EgoGoalSet (fast_exploration_fsm.cpp)

### 三条决策路径

**路径 A — TSP 探索** (`callExplorationPlanner`, :2194):
```
planExploreTSP → 所有 frontier TSP → 选最佳视点
→ getAndPublishNextAim → pubLocalGoal
```

**路径 B — LLM 引导探索** (`callExplorationLLMPlanner`, :2210):
```
planLLMExploration(area_id) → 区域内 frontier TSP
→ getAndPublishNextAim → pubLocalGoal
```

**路径 C — 物体导航** (GO_TARGET_OBJECT):
```
scene_graph_->getPathToObjectWithId(object_id) → path_res_
→ getAndPublishNextAim → pubLocalGoal
```

**路径 D — 航点导航** (`goTargetWithWaypoint`, :1860):
```
astarSearch(cur_poly_, target_poly, path_res_)
→ getAndPublishNextAim → pubLocalGoal
```

### pubLocalGoal (:2172-2192)

```cpp
void FastExplorationFSM::pubLocalGoal(
    const Eigen::Vector3d local_goal,  // 目标位置
    const double yaw,                  // 目标偏航
    const bool look_forward,           // 朝向模式
    const uint8_t yaw_mode,            // NORMAL / LOW_SPEED / PANORAMA
    const uint8_t yaw_path_mode)       // SHORTEST / KEEP_DIRECTION
{
    quadrotor_msgs::EgoGoalSet msg;
    msg.drone_id = md_->drone_id_;
    msg.source_task_id = active_instruction_task_id_;
    msg.goal[0..2] = local_goal;
    msg.yaw = yaw;
    msg.yaw_mode = yaw_mode;
    // ...
    ego_goal_pub_.publish(msg);   // → /local_goal → EGOReplanFSM
}
```

---

## 关键数据结构回顾

```cpp
// 邻接边 — 定义骨架图的拓扑
class Edge {
    PolyHedronPtr poly_nxt_;           // 邻居 place
    double length_;                    // 边长度 [m]
    double weight_;                    // 权重（当前未用于 A*）
    std::vector<Eigen::Vector3d> path_; // 边内路径点（当前未用于 A*）
};

// place 节点
class Polyhedron {
    Eigen::Vector3d center_;           // A* 使用的节点位置
    std::vector<Edge> edges_;          // ← A* 遍历的邻接边表
    std::vector<PolyHedronPtr> connected_nodes_; // 含 gate（A* 不使用）
};

// 物体到 place 的绑定
struct ObjectEdge {
    PolyHedronPtr polyhedron_father;   // 物体挂载的 place（可为 nullptr）
};

struct ObjectNode {
    ObjectEdge edge;                   // 每个物体通过 edge 绑定到 0 或 1 个 place
    Eigen::Vector3d pos;               // 物体世界坐标
};
```
