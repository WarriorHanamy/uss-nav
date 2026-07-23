# OPT_CASES.md — SUPER 优化难例挖掘与并行扫参

> 从端到端 trace 中自动感知"不好解"的优化问题，抽离 cost function 的问题构建，
> 以 SUPER planning 的问题形式对 corner case 做离线并行参数测试。

## 流水线总览

```
super trace (case_dump.enable=true)
  │  SuperPlanner 检测难例 → dump case 目录 + ROS log event=case_dumped
  │  Fluent Bit → opt_events.ltsv (机器可读事件流)
  ▼
mine_cases.py   trace 事件 × case 目录 → dataset manifest (JSONL)
  ▼
opt_case_replay ROS-free C++ 二进制: case → ExpTrajOpt::optimize → result.yaml
  ▼
sweep.py        cases × param grid × N 并行容器 → results.csv + report.md
```

## 1. 难例检测与 dump（在线）

`bringup_test/params/super_planner/click_smooth_ros1.yaml` 的 `case_dump:` 块控制：

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable` | false | 总开关 |
| `output_dir` | `/tmp/opt_cases` | case 输出根目录（sim 配置为 `/workspace/.artifacts/opt_cases`) |
| `dump_on_overtime` | true | replan 超时即 dump |
| `slow_iter_num` | 2000 | 成功但 L-BFGS cost 调用次数 ≥ 阈值 → dump(slow_convergence) |
| `pos_penna_warn` | 0.1 | 成功但 corridor penalty ≥ 阈值 → dump(high_violation，失败线为 0.2) |
| `consec_failures` | 3 | 连续 replan 失败 N 次后，下一次 replan 强制 dump(repeated_failure) |
| `min_interval_s` | 1.0 | 两次 dump 的最小墙钟间隔（防持续失败刷屏）[s] |
| `max_cases` | 200 | 单进程 dump 总数上限（0 = 不限） |
| `slow_opt_ms` | 15.0 | 成功但 exp 优化墙钟超阈 → dump(slow_opt) |
| `slow_sfc_ms` | 10.0 | exp SFC 构建墙钟超阈 → dump(slow_sfc，成败均触发） |

## 逐次 replan 计时（replan_timing)

`ReplanOnce`/`PlanFromRest` 每次退出（RAII，覆盖所有 return 路径）输出：

```
[SUPER][Progress] event=replan_timing stage=ReplanOnce ret=0 total_t=... frontend_t=... \
    exp_sfc_t=... exp_opt_t=... back_frontend_t=... back_sfc_t=... back_opt_t=... \
    viz_t=... goal_shift_t=... iter_num=... final_cost=...
```

- `exp_sfc_t`/`back_sfc_t`：两条 SFC 构建（SearchPolytopeOnPath / GeneratePolytopeFromLine）单独计时
- `goal_shift_t`:PlanFromRest 起点最近空闲格搜索
- backup frontend 全路径计时（含 NO_NEED/FINISH 早退）
- `mine_cases.py` 自动聚合该事件输出 mean/std/p50/p90/p99/max 分段表

## backup 段与 backup replay

case.yaml 的 `backup:` 段保存 backup 优化问题（t0/te/heu_ts/heu_dur/heu_p/单 polytope SFC),
replay 在 exp 轨迹 replay 成功后自动续跑 backup 优化，输出 `backup.success/opt_time/opt_times`。

`super_planner/backup_reopt_en`（默认 false)：遗留的 backup 二次热启动优化（结果被丢弃），
每次 replan 白跑一次完整 L-BFGS，已默认关闭。

## L-BFGS 参数

`traj_opt/<ns>/lbfgs_mem_size`(256)、`lbfgs_past`(3)、`lbfgs_max_iterations`(0 = 不限），
exp_traj / backup_traj namespace 独立，原为硬编码，现已可配置可扫参。

触发原因（`reason` 字段，可组合）:`exp_opt_failed`、`replan_overtime`、
`slow_convergence`、`high_violation`、`repeated_failure`、`sfc_failed`(frontend)。

每次 dump 产生：

```
.artifacts/opt_cases/<case_id>/
├── case.yaml             # 问题定义: head/tail PVAJ, guide_path+t, pass_wps, SFC 平面, 触发原因, 优化结果
├── cloud.pcd             # SFC 搜索用的局部点云(用于离线重生成 SFC)
└── config_snapshot.yaml  # 运行时 planner 配置原文
```

同时写 ROS log(`fluentbit_roslog.log` / `opt_events.ltsv` 可检索）:

```
[SUPER][Progress] event=case_dumped case_id=case-... reason=exp_opt_failed stage=backend dir=...
```

## 2. 机器可读事件流（Fluent Bit)

- `docker/fluent-bit-roslog-fields.lua` 解析 `-- [SUPER][Progress] event=xxx k=v` 双 tag 格式
  （兼容无前缀的 `[Module] event ...` 单 tag 格式），提取 `module`/`event`/全部 key=value 字段
  (whitelist: MissionFSM, SceneGraph, GlobalBelief, EGOPlanner, EGOOptimizer, SUPER, ExpOpt, Fsm)。
- `fluent-bit.conf` 用两条 `rewrite_tag` filter（单 tag match，多 pattern match 实测不生效）
  把含 `module` 字段的结构化事件复制到 `uss_nav.opt_event` 流，以 LTSV 写入 trace 目录的
  `opt_events.ltsv`(file 输出不支持 json_lines,LTSV 保留全部动态字段），不依赖 OpenSearch。

## 3. 挖掘 case(mine_cases.py)

```bash
uv run --script tools/opt_cases/mine_cases.py \
    --trace .artifacts/traces/super-20260722-101530
# → .artifacts/opt_cases/manifest-super-20260722-101530.jsonl + 终端摘要
```

## 4. 单 case replay(opt_case_replay)

ROS-free 二进制（`planner/Apps/opt_case_replay.cpp`)，容器内运行：

```bash
docker run --rm --entrypoint bash \
  -v $PWD/.artifacts/devel:/workspace/devel \
  -v $PWD/.artifacts:/workspace/.artifacts \
  -v $PWD/ws_main/src:/workspace/src:ro \
  ego-planner-sim -c "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && \
    /workspace/devel/lib/super_planner/opt_case_replay \
      --case /workspace/.artifacts/opt_cases/<case_id> \
      --override /workspace/.artifacts/ovr.yaml \
      --regen-sfc --repeat 3"
```

| 参数 | 说明 |
|------|------|
| `--case DIR` | case 目录（必需） |
| `--override FILE` | 参数覆盖 yaml（与主配置同结构，可多次） |
| `--regen-sfc` | 用 cloud.pcd 经 ROGMapOffline + CorridorGenerator 重生成 SFC；默认用 dump 的 SFC |
| `--repeat N` | 重复优化 N 次取最快耗时 |
| `--out FILE` | result.yaml 输出路径（默认 stdout) |

输出 `result.yaml`: `ret / lbfgs_ret / iter_num / final_cost / opt_time / penalty_log /
sfc_count / sfc_regenerated / traj_duration / exp_opt_times[] / sfc_time(regen 时) /
backup{success, opt_time, opt_times[]}`。

override 合并规则：以 `config_snapshot.yaml` 为底，按路径覆盖（如 `traj_opt/exp_traj/penna_t`),
并强制 `rog_map/ros_callback/enable=false`、`case_dump/enable=false`。

## 5. 并行扫参(sweep.py)

```bash
uv run --script tools/opt_cases/sweep.py \
    --cases .artifacts/opt_cases \
    --grid tools/opt_cases/grid_example.yaml \
    --parallel 8 \
    --out .artifacts/sweeps/sweep-001
```

- grid yaml 的 `params:` 以 `a/b/c: [v1, v2]` 形式给出，取笛卡尔积（见 `grid_example.yaml`)
- 每个 (case, combo) 起一个短命容器跑 `opt_case_replay`，结果聚合为
  `results.csv`（全字段）和 `report.md`
- report 分两部分：per-combo 跨 case 聚合（success rate → opt_time std → mean 排序，
  含 opt/sfc/backup 分段的 mean/std/p90)+ per-case 明细表
- `--repeat N`（默认 5）取 best-of-N 为 opt_time，全部 N 次时间序列写入 result
- `--cpuset 4-9` 把 replay 容器绑核，配合 `--parallel <= 核数`，避免计时被宿主机竞争污染
- `--regen-sfc` 对所有任务重生成 SFC（保真度更高，显著更慢）

## 已知限制

- `--regen-sfc` 的保真度依赖 dump 点云对走廊生成查询的覆盖；`cloud.pcd` 只含 SFC 搜索
  收集到的局部点云，远离 guide path 的区域与在线地图不同。
- frontend 失败(`sfc_failed`）的 case 无 SFC,replay 必须配 `--regen-sfc`。
- 点云较大的 case 会明显增加 `--regen-sfc` 耗时（地图更新 + raycast)。
