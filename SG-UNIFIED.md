# SG-Unified — 时空语义统一绑定层

> 场景图第三数据流与算法：将空间骨架（多面体图）与语义物体（CLIP 特征节点）绑定为统一的时空语义场景图，支持零样本区域推理与导航决策。

---

## 1. 概述

场景图（SceneGraph）由三个层次构成：

| 层次 | 组件 | 数据 | 角色 |
| --- | --- | --- | --- |
| 空间层 | `skeleton_generation` | `Polyhedron` 节点 + `Edge` 边 | 自由空间拓扑图 |
| 语义层 | `object_factory` | `ObjectNode` + CLIP 512-d 特征 | 物体检测与跟踪 |
| **统一绑定层** | `scene_graph` + `skeleton_cluster` | `ObjectEdge` + `PolyhedronCluster` | **将物体挂载到多面体，多面体聚合成区域，区域获得语义标签** |

统一绑定层的核心任务：
1. **对象-骨架绑定**：将检测到的物体通过 `ObjectEdge::polyhedron_father` 挂载到所属多面体
2. **空间区域聚类**：多面体聚合成语义区域（房间），物体随父节点归入对应区域
3. **区域语义标注**：通过 LLM 零样本推理为区域添加 `room_label_` / `room_description_`
4. **分层可视化**：三层图结构（顶层 → 房间层 → 物体层）用于监控与调试

---

## 2. 核心数据结构

### 2.1 `ObjectEdge` — 对象-骨架绑定边

`data_structure.h:110-120`

```cpp
struct ObjectEdge {
  enum EdgeType {
    UNKNOWN = 0,
    WITH_SKELETON = 1,  // 挂载到多面体
    WITH_OBJECT = 2     // 物体间关系
  } father_type{UNKNOWN};
  PolyHedronPtr polyhedron_father{nullptr};   // 所属空间多面体
  ObjectNodePtr object_father{nullptr};        // 所属父物体
  std::vector<ObjectNodePtr> object_child;     // 子物体列表
};
```

`ObjectEdge` 是统一图的核心连接结构。每一个 `ObjectNode` 通过 `edge.polyhedron_father` 指针关联到骨架图中的一个 `Polyhedron` 节点，形成**空间-语义混合图**的边。

### 2.2 `ObjectNode` — 语义物体节点

`data_structure.h:129-166`

```cpp
struct ObjectNode {
  int id;                           // 唯一物体 ID
  std::string label;                // 语义标签
  Eigen::VectorXd label_feature;    // CLIP 512-d 语义特征向量
  Eigen::Vector3d pos;              // 物体中心世界坐标 [m]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  // 物体点云
  ObjectEdge edge;                  // 绑定到骨架或其它物体
  // 过滤状态
  bool is_alive{true};
  ros::Time last_detection_time;
  unsigned int detection_count{0};
};
```

### 2.3 `Polyhedron` — 自由空间多面体节点

`data_structure.h:296-355`

```cpp
class Polyhedron {
  Eigen::Vector3d center_;           // 多面体质心 [m]
  std::vector<PolyhedronFtrPtr> ftrs_;  // 边界前沿
  std::vector<PolyHedronPtr> connected_nodes_;  // 连接的邻接多面体
  std::vector<Edge> edges_;          // 骨架图边

  std::map<int, ObjectNodePtr> objs_;  // <== 挂载在此多面体上的物体
  int area_id_{-1};                    // <== 所属区域 ID
};
```

### 2.4 `PolyhedronCluster` — 区域/房间

`data_structure.h:464-520`

```cpp
class PolyhedronCluster {
  std::vector<PolyHedronPtr>   polys_;    // 属于该区域的多面体
  std::vector<ObjectNodePtr>  objects_;   // 属于该区域的物体
  std::string room_label_, room_description_;  // LLM 标注的区域类型
  Eigen::Vector3d box_min_, box_max_;     // AABB 包围盒 [m]
  Eigen::Vector3d center_;                // 区域质心 [m]
  std::map<int, bool> nbr_area_;          // 邻接区域拓扑
};
```

### 2.5 统一图结构示意

```
                     ┌───────────────────┐
                     │   场景图 (SG)       │
                     │  scene_graph.cpp    │
                     └──────┬────────────┘
                            │ 包含
              ┌─────────────┼──────────────┐
              ▼             ▼              ▼
   ┌──────────────────┐ ┌──────────┐ ┌───────────┐
   │ AreaHandler      │ │ Skeleton │ │ Object    │
   │ area_map_        │ │ Generator│ │ Factory   │
   │ (区域聚类)        │ │ (多面体)  │ │ (物体检测)  │
   └────────┬─────────┘ └────┬─────┘ └─────┬─────┘
            │                │              │
            │  ┌─────────────┼──────────────┐
            │  ▼             ▼              ▼
            │  Polyhedron    Polyhedron     ObjectNode
            │  area_id_=0    area_id_=1     edge.polyhedron_father
            │                              → Polyhedron[0]
            ▼
        PolyhedronCluster
        id_=0, room_label_="kitchen"
        polys_ [P0, P1, P2]
        objects_ [chair, table, cup]
```

---

## 3. 绑定算法流程

### 3.1 物体挂载到多面体

数据流：

```
YOLOE + CLIP → EncodeMask.msg → ObjectFactory → ObjectNode
                                                    │
                                                    ▼
                                    ObjectEdge::polyhedron_father
                                                    │
                                        ┌───────────┴──────────┐
                                        ▼                     ▼
                                  Polyhedron::objs_     Polyhedron::area_id_
                                        │                     │
                                        ▼                     ▼
                                  PolyhedronCluster::objects_
```

`ObjectFactory` 内部在 `processSingleObject()` 和 `mergeObjectIntoMap()` 中完成挂载，关键逻辑：

- 每个新物体/更新物体调用 `skeleton_gen_->mountTopoPoint(obj->pos)` 获得所属多面体
- 结果存入 `obj->edge.polyhedron_father`
- 多面体的 `objs_` 映射当前仅用于内部，**真正挂载到区域由 `updateObjectToSceneGraph()` 完成**

### 3.2 `updateObjectToSceneGraph()` — 统一更新

`scene_graph.cpp:53-82`

```
场景图更新循环 (每个 navigation step):

  1. doDenseCheckAndExpand()  → 新多面体
       │
       ▼
  2. incrementalUpdateAreas() → 更新区域聚类
       │
       ▼
  3. updateObjectToSceneGraph()
       │
       ├── 清空所有区域 objects_
       ├── 遍历 object_factory_->object_map_
       │    对每个物体:
       │      area_id = obj->edge.polyhedron_father->area_id_
       │      area_map_[area_id]->addObject(obj)
       │
       └── 检测区域物体数变化
            触发 areas_need_predict_ 标记
       │
       ▼
  4. visualizeSceneGraph() → 三层可视化
```

挂载更新当前是全量遍历（非增量），代码中明确标注：
> `// todo [gwq] 物体挂载更新可以改成增量式的，但是我懒了，先这样吧 :D`

### 3.3 路径规划：物体→路径

`scene_graph.cpp:85-119` — `getPathToObjectWithId()`

```
查询目标物体 ID
  → 获取 obj->edge.polyhedron_father (目标区域的多面体)
  → 获取 cur_poly_ (无人机当前所在多面体)
  → skeleton_gen_->astarSearch(start, goal, path)
      → 在骨架多面体图上执行 A* 搜索
  → 返回路径点序列 + 末端 yaw
```

---

## 4. 区域聚类算法

### 4.1 谱聚类 (Spectral Clustering)

`skeleton_cluster.cpp:6-247` — `SpectralCluster::calculate()`

| 步骤 | 函数 | 说明 |
| --- | --- | --- |
| 1. 相似度矩阵 | `calSimilarityMatrix()` | 基于多面体邻接性 + 高斯核距离 |
| 2. 度矩阵 | `calDegreeMatrix()` | 对角线 = 相似度行和 |
| 3. 拉普拉斯矩阵 | `calLaplacianMatrix()` | 归一化拉普拉斯 L = I - D⁻¹ᐟ² W D⁻¹ᐟ² |
| 4. 特征分解 | `calLaplacianEigen()` | 实对称矩阵 Eigensolver → 特征值间隙自动选 k |
| 5. K-Means | `kmeans()` | 对特征向量行向量聚类 |

**相似度计算** (`calSimilarityMatrix`, L67):

```cpp
W(i, j) = exp(-eula_dis² / (2 * 0.1))   // 仅对邻接多面体计算
```

**自动选 k** (`calLaplacianEigen`, L113-127):

```cpp
k = argmax(eigen_values[i+1] - eigen_values[i])  // 最大特征值间隙
```

> 注意：当前代码中 `k_ = 5` 的硬编码覆盖了自动选 k 的结果（L129），用于稳定实验。

### 4.2 增量社区检测 (Leiden / Louvain)

`skeleton_cluster.cpp:290-383` — `AreaHandler::communityDetection()`

当新多面体加入时，并非重新聚类全局，而是**增量更新**涉及的区域：

```
incrementalUpdateAreas(new_polys):
  │
  ├── 找到与新多面体邻接的旧区域 (areas_to_update)
  │
  ├── candidate_polys = 旧区域多面体 + 新多面体
  │
  ├── communityDetection(candidate_polys)
  │     ├── 基于 igraph 构建图
  │     ├── Leiden CPM Vertex Partition (resolution=0.018)
  │     ├── 边界多面体合并 (MERGE_THRESHOLD=4)
  │     └── 匈牙利匹配 new→old areas
  │
  └── 更新/创建/删除区域
```

**图构建** (`communityDetection`, L314-338):

- 节点：非 gate 多面体
- 边：`polys[i]->edges_` 中的邻接关系
- 权重：邻接边 `weight=1.0`，强制连接边 `weight=1e-9`

**后合并逻辑** (L496-564):

社区检测后，统计社区间边界多面体对数，超过阈值（`MERGE_THRESHOLD=4`）即合并两个社区。用并查集（DSU）实现。

---

## 5. LLM 零样本推理

### 5.1 序列化：场景图 → JSON Prompt

五种 Prompt 类型的序列化方法：

| 方法 | 文件行 | Prompt 类型 | 输入 | 输出 |
| --- | --- | --- | --- | --- |
| `singleRoomPredictionPromptGen()` | `scene_graph.cpp:121` | `ROOM_PREDICTION` | 单区域 ID | 物体列表 + 尺寸 |
| `newAreaPredictionPromptGen()` | `scene_graph.cpp:152` | `ROOM_PREDICTION` | 待预测的区域集合 | 多区域 JSON |
| `chooseAreaToGoPromptGen()` | `scene_graph.cpp:193` | `EXPL_PREDICTION` | 全区域+访问历史+目标 | 区域+邻居+前沿数 |
| `chooseTerminateObjIdPromptGen()` | `scene_graph.cpp:243` | `TERMINATE_OBJ_ID` | 当前区域物体列表 | 物体 ID+标签+位置 |
| `DFDemoPromptGen()` | `scene_graph.cpp:271` | `DF_DEMO` | 全区域+物体 | 区域+物体列表 |

JSON 输出示例 (`singleRoomPredictionPromptGen`, L121-150):

```json
{
  "areas": [{
    "id": "0",
    "dimensions": {
      "width": "5.23",
      "height": "4.11",
      "unit": "meters"
    },
    "objects": [
      {"info": "chair,3.45,2.10"},
      {"info": "table,4.02,1.55"}
    ]
  }]
}
```

### 5.2 反序列化：JSON → 场景图标注

`handleRoomPredictionResult()` (`scene_graph.cpp:503-545`):

```
LLM 回复 JSON → 解析 areaType + description
  → area_map_[area_id]->room_label_ = areaType     // "kitchen"
  → area_map_[area_id]->room_description_ = description
  → visualizeSceneGraph() 触发刷新
```

### 5.3 Prompt 通信架构

```
SceneGraph (C++)
  │  prompt_pub_  →  /scene_graph/prompt    [PromptMsg]
  │                                           ├── prompt_id
  │                                           ├── prompt_type
  │                                           ├── prompt (JSON 序列化的场景图)
  │                                           └── option: SEND_PROMPT
  ▼
LLM_interface_thread.py (Python ROS node)
  │  Qwen / DeepSeek / Doubao API
  ▼
  │  llm_ans_sub_  ←  /scene_graph/llm_ans   [PromptMsg]
  │                                           ├── prompt_id
  │                                           ├── answer (LLM 回复)
  │                                           └── option: SEND_ANSWER
  ▼
SceneGraph (C++)
  → llmAnsCallback()  → std::promise 触发
  → handleRoomPredictionResult() 标注区域
```

---

## 6. 三层层次可视化

`scene_graph.cpp:685-849` — `visualizeSceneGraph()`

```
顶层 (Top Level):     白色球体 @ 所有区域质心均值     z=13m
  │
  ├── 房间层 (Room Level):  彩色球体 @ 每个区域质心    z=7m
  │    │
  │    ├── 区域边界框:     LINE_LIST, 区域颜色
  │    ├── 区域间连线:     邻接关系 LINE_LIST
  │    └── 区域标签:       TEXT_VIEW_FACING "Area[0]kitchen"
  │
  └── 物体层 (Object Level): 物体→区域连线            z=2m
       │
       └── 物体↔房间边:     LINE_LIST, 区域颜色
```

通过 RViz 订阅 `/scene_graph/vis` (MarkerArray) 查看。

---

## 7. ROS 话题接口

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/scene_graph/prompt` | `scene_graph/PromptMsg` | 输出 | 序列化场景图 → LLM |
| `/scene_graph/llm_ans` | `scene_graph/PromptMsg` | 输入 | LLM 回复 → 场景图 |
| `/scene_graph/vis` | `visualization_msgs/MarkerArray` | 输出 | 三层层次可视化 |
| `/skeleton/cluster_vis` | `visualization_msgs/MarkerArray` | 输出 | 区域聚类可视化 |

### `PromptMsg` 消息定义

`scene_graph/msg/PromptMsg.msg`

```yaml
Header header
uint32 prompt_id
uint8  option                # SEND_PROMPT=0 / SEND_ANSWER=1
uint8  prompt_type           # 0:ROOM_PREDICTION ... 20:TASK_OVER_PREDICTION
string prompt                # 输入：JSON 序列化的场景图
string answer                # 输出：LLM 回复
```

---

## 8. 完整数据流

```
[传感器]                    [感知]                     [场景图]
RGB-D + IMU               YOLOE + MobileCLIP         ObjectFactory
    │                          │                         │
    ▼                          ▼                         ▼
sensor_msgs::Image        EncodeMask.msg            ObjectNode map
                                                    (CLIP 512-d 特征)
                                                         │
                                                         ▼
[空间骨架]                                             挂载
ESDF Map ──► SkeletonGenerator          ┌─── ObjectEdge::polyhedron_father
    │           │                       │
    ▼           ▼                       ▼
Polyhedron    Polyhedron             PolyhedronCluster
 (拓扑图)       (当前位姿挂载)         (区域聚类)
                              │       │
                              ▼       ▼
                           [推理]   [导航]
                        LLM Prompt  A* Search
                        ────────    ────────
                        room_label  path to
                        = "living   object
                         room"
```

---

## 9. 关键代码索引

| 文件 | 行 | 内容 |
| --- | --- | --- |
| `include/scene_graph/data_structure.h` | 110-120 | `ObjectEdge` — 统一绑定边 |
| `include/scene_graph/data_structure.h` | 129-166 | `ObjectNode` — 语义物体节点 |
| `include/scene_graph/data_structure.h` | 296-355 | `Polyhedron` — 空间多面体 |
| `include/scene_graph/data_structure.h` | 464-520 | `PolyhedronCluster` — 区域 |
| `include/scene_graph/scene_graph.h` | 50-204 | `SceneGraph` 顶层编排器 |
| `src/scene_graph.cpp` | 36-51 | `updateSceneGraph()` — 统一更新入口 |
| `src/scene_graph.cpp` | 53-82 | `updateObjectToSceneGraph()` — 物体→区域同步 |
| `src/scene_graph.cpp` | 85-119 | `getPathToObjectWithId()` — 物体路径规划 |
| `src/scene_graph.cpp` | 121-150 | `singleRoomPredictionPromptGen()` — 单区域序列化 |
| `src/scene_graph.cpp` | 193-241 | `chooseAreaToGoPromptGen()` — 探索决策序列化 |
| `src/scene_graph.cpp` | 243-269 | `chooseTerminateObjIdPromptGen()` — 目标物体选择 |
| `src/scene_graph.cpp` | 503-545 | `handleRoomPredictionResult()` — LLM 标注回写 |
| `src/scene_graph.cpp` | 685-849 | `visualizeSceneGraph()` — 三层可视化 |
| `include/scene_graph/skeleton_cluster.h` | 36-64 | `SpectralCluster` 谱聚类 |
| `include/scene_graph/skeleton_cluster.h` | 74-128 | `AreaHandler` 增量区域管理 |
| `src/skeleton_cluster.cpp` | 6-24 | `SpectralCluster::calculate()` 聚类流程 |
| `src/skeleton_cluster.cpp` | 290-383 | `AreaHandler::communityDetection()` 社区检测 |
| `src/skeleton_cluster.cpp` | 410-658 | `AreaHandler::incrementalUpdateAreas()` 增量更新 |
| `include/scene_graph/object_factory.h` | 73-246 | `ObjectFactory` 物体检测与融合 |
| `msg/PromptMsg.msg` | 1-31 | Prompt 消息定义 |
| `scripts/vla_swarm_prompt_router.py` | — | 20 种 Prompt 类型路由 |

---

## 10. 与 EGO-Planner 的集成

统一场景图在 `FastExplorationFSM` (12 状态 FSM) 中使用：

```
状态                       场景图调用
──────────────────────────────────────────────────
SG_INIT                   scene_graph_->initSceneGraph()
SG_UPDATE                 scene_graph_->updateSceneGraph()
SG_MOUNT_CUR_POLY         scene_graph_->mountCurPoly()
LLM_AREA_CHOOSE           scene_graph_->chooseAreaToGoPromptGen() + sendPrompt()
LLM_TERMINATE_OBJ_CHOOSE  scene_graph_->chooseTerminateObjIdPromptGen() + sendPrompt()
GO_TARGET_AREA            frontier_manager_->planLLMExploration()
GO_TARGET_OBJECT          scene_graph_->getPathToObjectWithId()
```

`fast_exploration_fsm.h` 持有 `SceneGraph::Ptr` 和 `CountingSceneGraph::Ptr`，统一场景图作为探索状态机的**环境表征核心**驱动所有决策。
