# Build Log

## future works:

### 1. Add Astar algorithm to verify path in the map between two polyhedron centers.

### 2. 根据多面体之间的链接关系，生成可行的骨架，并加入可行性判断与删除机制

### 3. 完善更新逻辑顺序，重点是什么时候更新

### 4. 优化计算速度（使用并行计算？剪枝？）

## VLA_Swarm Prompt

当前维护的处理端是：

- `scripts/LLM_interface_thread.py`：OpenAI 兼容接口。
- `scripts/LLM_interface_deepseek_thread.py`：DeepSeek Anthropic 兼容接口。
- `scripts/vla_swarm_prompt_router.py`：共享 Prompt 路由、模板加载和结构化错误。

VLA_Swarm `prompt_type=6-20` 的模板位于 `prompts_definition_swarm/`。文本类型可以直接调用模型；
依赖 SmallMap 或 Observation 的类型在对应数据管线接入前返回：

```json
{
  "success": false,
  "error": "observation_not_ready",
  "detail": "..."
}
```

所有答案都会保留原请求的 `prompt_id` 和 `prompt_type`，并设置 `option=SEND_ANSWER`。
API key 必须通过私有 ROS 参数 `~api_key` 或环境变量提供：

```bash
export SCENE_GRAPH_OPENAI_API_KEY=...
export SCENE_GRAPH_DEEPSEEK_API_KEY=...
```

`scripts/LLM_interface.py` 使用过时的消息导入和 Prompt 常量，已经废弃，不应作为启动入口。
