# Build Log

## future works:

### 1. Add Astar algorithm to verify path in the map between two polyhedron centers.

### 2. 根据多面体之间的链接关系，生成可行的骨架，并加入可行性判断与删除机制

### 3. 完善更新逻辑顺序，重点是什么时候更新

### 4. 优化计算速度（使用并行计算？剪枝？）

## VLA_Swarm Prompt

当前维护的文本处理端是：

- `scripts/LLM_interface_thread.py`：OpenAI 兼容接口。
- `scripts/LLM_interface_deepseek_thread.py`：DeepSeek Anthropic 兼容接口。
- `scripts/LLM_interface_local_thread.py`：本地 OpenAI 兼容文本/视觉双端点。
- `scripts/vla_swarm_prompt_router.py`：共享 Prompt 路由、模板加载和结构化错误。

VLA_Swarm `prompt_type=6-20` 的模板位于 `prompts_definition_swarm/`。文本类型可以直接调用模型。
`scripts/LLM_interface_thread.py` 是 uss-nav 专用的纯文本节点，不订阅
`/vla_swarm/small_map` 或 `/vla_swarm/observation`。需要图像的 Prompt 会返回结构化错误，
不会退化为纯文本。

VLA-Swarm 的视觉职责由独立的 `LLM_interface_local_thread.py` 承担，不写入
`LLM_interface_thread.py`。默认路由如下：

```text
纯文本 Prompt                         -> http://127.0.0.1:2233/v1
SmallMap / Observation 视觉 Prompt    -> http://127.0.0.1:2235/v1
```

启动方式：

```bash
python3 scripts/LLM_interface_local_thread.py
```

可配置参数包括：

```text
~text_base_url=http://127.0.0.1:2233/v1
~vision_base_url=http://127.0.0.1:2235/v1
~text_model=
~vision_model=
~api_key=EMPTY
~small_map_topic=/vla_swarm/small_map
~small_map_max_age=5.0
~observation_topic=/vla_swarm/observation
~observation_max_age=30.0
~request_timeout=120.0
~temperature=0.2
```

模型参数为空时，节点分别调用两个端点的 `models.list()` 获取第一个模型。该节点会严格按
session、Observation 批次和序号缓存图像，缺图时返回结构化错误。

三个 LLM 节点订阅相同 Prompt 话题，因此运行时只能启动一个。

所有答案都会保留原请求的 `prompt_id` 和 `prompt_type`，并设置 `option=SEND_ANSWER`。
API key 必须通过私有 ROS 参数 `~api_key` 或环境变量提供：

```bash
export SCENE_GRAPH_OPENAI_API_KEY=...
export SCENE_GRAPH_DEEPSEEK_API_KEY=...
```

`scripts/LLM_interface.py` 使用过时的消息导入和 Prompt 常量，已经废弃，不应作为启动入口。
