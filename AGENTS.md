# AGENTS.md — 项目文档与工具索引

> 供 AI agent 快速了解 USS-NAV 的文档体系和可视化工具链。

## 文档索引（根目录 *.md）

| 文件 | 说明 |
|------|------|
| `README.md` | 项目概述、工作区结构、核心模块说明、构建与部署指南 |
| `VIEW.md` | 架构总览入口，包含 SceneGraph 和 EGO Planner 章节概要，多文档导航 |
| `EGO.md` | EGO-Planner 实时轨迹优化文档：12 状态 FSM、算法管道、ROS 话题接口、消息契约、代码组织 |
| `SCENEGRAPH.md` | SceneGraph 上层环境表征：骨架生成、物体管线、区域聚类、LLM 交互、API 参考 |
| `NEXT_SCENEGRAPH.md` | SceneGraph 重构提案：问题分析、目标架构、迁移计划 |
| `CODEBASE.md` | 全量代码库参考：仓库结构、三层架构、62 个 ROS 消息定义、算法与数据流 |
| `instruction_description.md` | Instruction.msg 字段映射参考：12 种 Instruction 类型及各字段含义 |

### 文档渲染

`/tools/md2html/` — 将 Markdown 文档渲染为单页 HTML（语法高亮、Katex 公式、Mermaid 图表、TOC 侧边栏、暗色模式）。
使用 Bun 运行，支持单文档和多标签渲染（`VIEW.md` 使用多标签模板 `template_tabs.html`）。

```bash
cd tools/md2html && bun render.ts ../EGO.md     # 渲染单文档
cd tools/md2html && bun render.ts --tabs         # 多标签渲染 VIEW
```

## Web 可视化工具

| 工具 | 路径 | 技术栈 | 说明 |
|------|------|--------|------|
| `md2html` | `tools/md2html/` | Bun + marked + highlight.js + Katex + Mermaid | Markdown → HTML 文档渲染 |
| `map-demo` | `tools/map-demo/` | Bun + Three.js + OrbitControls | 3D 地图可视化：占据网格、未知空间、ESDF 热力图、多面体骨架、UniformGrid 前沿单元，含交互式图层切换 |
| Docker 容器 | `tools/infra.ts` | Bun + Docker | `ego-planner-sim` 容器管理脚本（构建、运行、日志、清理），在容器中启动 RViz |
| WebSocket 服务 | `ws_main/src/script/tcp.py` | Python + websockets | 远程无人机控制服务，监听 `192.168.100.124:8080`，支持启停、起飞、降落、探索等命令 |

## Web 可视化使用方式

```bash
# 渲染文档为 HTML，用浏览器打开
cd tools/md2html && bun render.ts ../EGO.md && xdg-open ../EGO.html

# 启动 3D 地图可视化
cd tools/map-demo && bun render.ts && xdg-open ../_site/map-demo/index.html

# 启动 Docker 仿真 RViz
bun run tools/infra.ts docker:run
```
