# SceneGraph Next — 职责边界与接口重构

> SceneGraph `next` 是对现有 SceneGraph 的职责清理——**SceneGraph 回归为语义地图构建管道 + 查询层，LLM 桥接职责提取到独立的 LLMBridge**。

---

## 现状问题

当前 SceneGraph 被塞入了两个本质上不相干的职责：

| 职责族 | 当前实现位置 | 问题 |
|--------|------------|------|
| 语义地图构建 | `SkeletonGenerator`, `ObjectFactory`, `AreaHandler` | ✅ 正确 |
| LLM Prompt 生成 | `*PromptGen()` ×7 方法 | ❌ 职责越界 |
| LLM 通信 | `sendPrompt()`, `hasPromptAnswer()`, `clearPromptData()` | ❌ 职责越界 |
| LLM 结果解析 | `handle*Result()` ×5, `parseVlaSwarmPromptResult()` | ❌ 职责越界 |
| FSM 直接穿透内部 | `scene_graph_->skeleton_gen_->area_handler_->area_map_` | ❌ 封装被破坏 |

此外存在两套并行的 LLM 等待模式（`THINKING` 状态手动 poll vs `VLA_SWARM` 的 `hasPromptAnswer()` 轮询），以及三个冗余的 Python LLM 节点。

---

## Next 架构

```
┌──────────────────────────────────────────────────────────────────┐
│                  FastExplorationFSM (决策编排)                      │
│  只通过标准接口调用 LLMBridge 和 SceneGraph，不穿透内部               │
└────┬────────────────────────┬───────────────────────────────────┘
     │                        │
     ▼                        ▼
┌──────────────┐     ┌──────────────────────┐
│ LLMBridge    │     │ SceneGraph           │ ←── 纯地图引擎
│ (LLM 桥接层)  │     │                      │
│              │     │ 主动管道:              │
│ promptGen()  │     │  PointCloud → ikdTree │
│ sendPrompt() │     │  → Polyhedron → Area  │
│ parseResult()│     │  → leiden clustering  │
│              │     │  Depth+semseg → Obj   │
│              │     │  → hungarian fusion   │
│              │     │  → ObjectMap          │
│              │     │                      │
│ 不拥有地图数据 │     │ 暴露查询接口:          │
│ 只读 SceneGraph│     │  getArea(id)          │
│ 格式化→LLM    │     │  getObjectsInArea()   │
│ LLM解析→FSM  │     │  getNeighborAreas()   │
└──────────────┘     │  getPathToObject()    │
                     │  getFrontierSet()     │
                     │  mountCurPoly()       │
                     └──────────────────────┘
                               ▲
                               │ (通过 MapInterface)
                     ┌─────────┴─────────┐
                     │  EGOReplanFSM      │
                     │  (共享 GridMap)    │
                     └───────────────────┘
```

### 关键边界

| 层面 | 拥有数据 | 主动做什么 | 暴露给谁 |
|------|---------|-----------|---------|
| **SceneGraph** | Polyhedron图, Area聚类, ObjectMap | 维护拓扑骨架、融合物体、增量聚类 | LLMBridge + FSM 通过查询 API |
| **LLMBridge** | 无持久数据 | 读 SceneGraph → 格式化 prompt → 异步 LLM → 解析结果 → 返回决策 | FSM |
| **FSM** | FSM 状态 (cur_state, aim_pose, ...) | 编排决策流程、调用 LLMBridge 和 SceneGraph | — |

---

## SceneGraph — 语义地图引擎

### 主动管道 (active pipeline)

SceneGraph 不是被动查询层——它持续将 raw sensor data 变换为结构化可查询数据：

```
LiDAR PointCloud
    ↓ FOV 滤波
    ↓ ikd-Tree 索引
    ↓ RayCasting + QuickHull → Polyhedron(凸多面体)
    ↓ Frontier 切分 + 扩展 → 邻接图
    ↓ Leiden 社区检测 → PolyhedronCluster(区域)
    ↓ 增量维护 → area_map_

RGB-D + YOLOE EncodeMask
    ↓ extractCloud (每 mask 按深度图提取点云)
    ↓ filteringCloud (半径 + voxel 滤波)
    ↓ Hungarian 匹配 → 跨帧融合 → ObjectMap
    ↓ 挂载到对应 Polyhedron 和 Area
```

这些变换在 `updateSceneGraph()` 中每帧执行，是算法的核心。

### 查询接口

```cpp
class SceneGraph {
    // ── 主动更新 (每帧调用) ──
    void updateSceneGraph(const Eigen::Vector3d &pos, double yaw);
    void mountCurPoly();

    // ── 拓扑查询 ──
    AreaData getArea(int area_id);                    // 区域 room_label, center, bbox, frontier 数
    std::vector<AreaData> getAllAreas();
    std::vector<int> getNeighborAreaIds(int area_id);
    PolyData getCurPoly();                            // 无人机当前 Polyhedron
    std::vector<PolyData> getPolyMap();               // 全部多面体

    // ── 物体查询 ──
    std::vector<ObjectData> getObjectsInArea(int area_id);
    std::vector<ObjectData> getObjectMap();            // 全部物体
    ObjectData getObjectById(int object_id);

    // ── 探索查询 ──
    std::vector<FrontierData> getFrontierSet();
    std::vector<Eigen::Vector3d> getPathToObject(int object_id);  // A* 拓扑路径

    // ── 序列化 ──
    bool saveMap(const std::string &path);
    bool loadMap(const std::string &path);
};
```

**输出数据结构：**

```cpp
struct AreaData {
    int id;
    std::string room_label, room_description;  // LLM 标注（由 LLMBridge 写入）
    Eigen::Vector3d center;
    Eigen::Vector3d bbox_min, bbox_max;
    int num_frontiers;
    std::vector<int> neighbor_area_ids;
    std::vector<int> object_ids;               // 区域内物体
};

struct ObjectData {
    int id;
    std::string label;
    double conf;
    Eigen::Vector3d pos;
    Eigen::Vector3d color;
    int poly_id;                               // 所属多面体
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  // 物体点云
};

struct FrontierData {
    Eigen::Vector3d position;
    int poly_id;
    int area_id;
    double exploration_gain;
};
```

### 关键设计约束

- SceneGraph **不知道** LLMBridge 的存在——LLMBridge 是 SceneGraph 的一个 consumer
- `room_label_` / `room_description_` 由 LLMBridge 通过 `updateAreaLabel(id, label)` 写入，不是 SceneGraph 自己产生
- 所有内部成员不再暴露 `skeleton_gen_->area_handler_->area_map_` 路径

---

## LLMBridge — LLM 桥接层

### 职责

| 职责 | 说明 |
|------|------|
| Prompt 生成 | 从 SceneGraph 查询 area/object/frontier 数据 → 格式化为 LLM JSON prompt |
| 通信管理 | 异步发送 prompt、等待响应（统一 future + timeout + retry 机制） |
| 结果解析 | 解析 LLM JSON 响应 → 返回结构化决策给 FSM |
| 状态维护 | prompt_id 管理、重试计数、超时策略 |

### 接口

```cpp
class LLMBridge {
    // ── 查询 SceneGraph（内部持有引用）─
    void bindSceneGraph(SceneGraph *sg);

    // ── prompt 生成 ──
    PromptTask roomPredictionPrompt(const std::vector<AreaData> &new_areas);
    PromptTask exploreChoicePrompt(const std::vector<AreaData> &all_areas);
    PromptTask terminateObjectPrompt(const std::vector<ObjectData> &objects,
                                     const std::string &instruction);
    PromptTask vlaSwarmPrompt(const std::string &task_command,
                              const SceneGraphStatus &status);

    // ── LLM 通信 ──
    void sendPrompt(PromptTask &task);            // 异步发送
    bool isAnswerReady(uint32_t prompt_id);       // 非阻塞检查
    bool waitForAnswer(uint32_t prompt_id,        // 阻塞等待(带超时)
                       int timeout_ms = 5000);

    // ── 结果解析 ──
    RoomPredictionResult parseRoomPrediction(const std::string &answer_json);
    ExploreChoiceResult parseExploreChoice(const std::string &answer_json);
    TerminateObjResult parseTerminateObject(const std::string &answer_json);
    VlaSwarmResult parseVlaSwarmResult(const std::string &answer_json);

    // ── 生命周期 ──
    void clear();
};
```

### 通信模式

```
LLMBridge                          Python LLM Node
    │                                    │
    │  publish PromptMsg                  │
    │  ──────────────────────────────►    │
    │  (prompt_id, type, prompt_json)    │
    │                                    │  LLM 推理
    │                                    │
    │  publish PromptMsg (answer)         │
    │  ◄──────────────────────────────   │
    │  (prompt_id, type, answer_json)    │
    │                                    │
    │  future.set_value(answer)           │
    │  waitForAnswer() 返回              │
    │  parseResult() → 决策              │
    │                                    │
```

### 消除 THINKING 状态

当前 FSM 的 `THINKING` 状态和 `VLA_SWARM_WAIT_LLM` 状态本质相同——都在等 LLM 响应。
Next 方案：**不需要专门的 Thinking State**。FSM 用非阻塞模式调用 LLMBridge：

```
handleVlaSwarmPlanLocal() {
    task = llm_bridge_.vlaSwarmPrompt(task_command, sg_status);
    llm_bridge_.sendPrompt(task);
    stash_ = VLA_SWARM_WAIT_LLM;  // 记录等待哪个状态
    transit(WORKING);
}

handleWorking() {                 // 10Hz 统一处理所有异步等待
    if (stash_ == VLA_SWARM_WAIT_LLM && llm_bridge_.isAnswerReady(task_id)) {
        result = llm_bridge_.parseVlaSwarmResult(...);
        transit(stash_);        // 切回目标状态
    }
}
```

即使无法消除 `WORKING` 状态（因为 FSM 需要 10Hz 持续执行其他工作），也只需要一个**统一的** `WORKING` 状态而非多个分散的 Wait 状态。

---

## FSM — 决策编排

### 变更

| 当前 | Next |
|------|------|
| 直接访问 `scene_graph_->skeleton_gen_->area_handler_->area_map_` | 通过 `scene_graph_->getArea(id)` / `getAllAreas()` |
| 手动调用 `chooseAreaToGoPromptGen()` + `sendPrompt()` + 结果存全局变量 | 调用 `llm_bridge_.exploreChoicePrompt()` + `sendPrompt()` + `parseExploreChoice()` |
| `THINKING` 状态直接 poll `llm_ans_str_poll_` | `WORKING` 状态统一调用 `llm_bridge_.isAnswerReady()` |
| 同时有 `handelThingkingProcess()` 和 `handleVlaSwarmWaitLLM()` 两套 | 统一异步等待 |
| 多个 VLA_SWARM 状态 (WAIT_LLM, WAIT_TARGET, APPROACH, YAW) 各有一个 handler | 精简为 `WORKING` + stash state 机制 |

### FSM 状态精简

```
当前:      INIT → WARM_UP → WAIT_TRIGGER → LLM_PLAN_EXPLORE → THINKING → ...
Next:      INIT → WARM_UP → WAIT_TRIGGER → LLM_PLAN_EXPLORE ──┐
                                                    ▲         │
                                                    │         ▼
                                                    └── WORKING (统一异步等待)
                                                    │         │
                                                    │         ▼
                                                    └── APPROACH_EXPLORE → ...
```

---

## Python LLM 节点统一

### 目标

从三个冗余节点 (`LLM_interface.py`, `LLM_interface_thread.py`, `LLM_interface_deepseek_thread.py`) 合并为一个基于配置的选择。

### 架构

```
LLM_interface.py (入口)
    ├── config: provider (openai / deepseek / aliyun)
    ├── prompt_router.py (来自现有 vla_swarm_prompt_router.py)
    ├── provider/
    │   ├── openai_provider.py
    │   ├── deepseek_provider.py
    │   └── aliyun_provider.py
    └── prompts_definition/ (system prompt 模板文件)
```

启动选择通过 ROS param 或环境变量：

```bash
rosrun scene_graph LLM_interface.py _provider:=deepseek
```

---

## 迁移路径

### Phase 1 — 提取接口定义

1. 定义 `SceneGraph::getArea()`, `getAllAreas()`, `getObjectMap()`, `getFrontierSet()` 等查询 API
2. 定义 `AreaData`, `ObjectData`, `FrontierData` 输出结构体
3. 定义 `LLMBridge` 类骨架（头文件 + 空实现）

### Phase 2 — 搬移 LLM 代码

1. 将 `*PromptGen()` 搬入 `LLMBridge::*Prompt()`
2. 将 `sendPrompt()` / `hasPromptAnswer()` 搬入 `LLMBridge`
3. 将 `handle*Result()` / `parseVlaSwarmPromptResult()` 搬入 `LLMBridge`
4. 统一 `sendPrompt()` 内部的 future + timeout 逻辑（消除 detached thread）

### Phase 3 — 清理 FSM

1. 替换 `scene_graph_->skeleton_gen_->area_handler_->area_map_` 为 `scene_graph_->getArea()` / `getAllAreas()`
2. 替换 LLM 调用的入口（FSM → LLMBridge）
3. 统一 `THINKING` / `VLA_SWARM_WAIT_LLM` 为 `WORKING` + stash
4. 删除 `handelThingkingProcess()` 中的手动 dispatch

### Phase 4 — Python 节点合并

1. 将 `vla_swarm_prompt_router.py` 通用化
2. 实现 provider 抽象层
3. 删除旧节点文件

### Phase 5 — 验证

1. 编译通过 (`catkin build` / `colcon build`)
2. `roscore` + `roslaunch ego_planner obj_nav.launch` 无启动错误
3. `rosrun scene_graph test_scene_graph` 单元测试通过（如存在）
4. LLM prompt/response 往返通信测试

---

## 增量 vs 全量

建议**增量式迁移**，每个 Phase 完成后保持兼容性：

- Phase 1-2: SceneGraph 同时保留旧方法（deprecated 标记）和新 API，LLMBridge 新加
- Phase 3: FSM 切换到新接口，旧 SceneGraph 方法逐步移除
- Phase 4: Python 节点可独立部署，旧节点保留作为 fallback

---

## 附录：不变的部分

以下不在此次重构范围内，保持不变：

- `MapInterface`（占用网格 + ESDF 共享指针）
- `EGOReplanFSM` 和轨迹规划管线
- `ObjectFactory` 的多线程管道（detection → filter → Hungarian fusion）
- `SkeletonGenerator` 的 RayCasting / QuickHull / Frontier 逻辑
- ROS 话题结构（`/scene_graph/prompt`, `/scene_graph/llm_ans`）
- `CountingSceneGraph`（轻量变体，保持独立）
