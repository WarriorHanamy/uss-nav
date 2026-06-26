---
name: ego-test-tdd
description: TDD (Red → Green → Refactor) workflow for EGO-Planner large-scale testing. Use when designing new test scenarios, debugging planner failures under parameter stress, or optimizing the test harness.
---

# EGO-Planner TDD Workflow

The three-phase cycle applies directly to ego-planner parameter sweep testing:

```
Red       Write a scenario the planner cannot solve
Green     Adjust parameters until the planner succeeds
Refactor  Optimize scope, parallelism, and reporting
```

---

## Phase 1: Red — Construct a Failing Scenario

Design a parameter combination that pushes the planner beyond its limits.

**Strategy**: max velocity too low, too many obstacles, or map too small.

```bash
# Write a failing scenario in scenarios.ts
# Example: low velocity + dense obstacles
{
  id: "edge_no_path",
  params: { max_vel: [0.15], obs_num: [80] },
  fixed: { flight_type: 2, max_acc: 0.3, x_size: 50, y_size: 30 },
  duration: 120,
}
```

**Run and observe failure**:

```bash
bun test:build
bun test:run edge_no_path
```

**Expected Red signals**:

| Signal                    | Where to look                                      |
| ------------------------- | -------------------------------------------------- |
| FSM enters EMERGENCY_STOP | `docker logs ego-test-edge_no_path` — grep for `EMERGENCY` |
| plan_result.status=false  | `_site/test-results/edge_no_path/plan_result.jsonl` |
| odom minimal movement     | Frontend 3D scene shows drone barely moving         |
| Container exits early     | `docker inspect` shows shorter-than-duration runtime |

**Threshold check (grep plan_result.jsonl)**:

```bash
grep false _site/test-results/edge_no_path/plan_result.jsonl \
  | wc -l
```

Count > 1 confirms planner failed — Red phase passes.

---

## Phase 2: Green — Make It Pass

Relax one or more parameters until the planner succeeds.

**Typical levers**:

| Lever        | Effect                            | Typical range       |
| ------------ | --------------------------------- | ------------------- |
| `max_vel`    | More time to find path            | 0.3 – 1.5 m/s       |
| `max_acc`    | More aggressive acceleration      | 0.5 – 2.0 m/s²      |
| `obs_num`    | Fewer obstacles                   | 10 – 40             |
| `x_size`     | More free space                   | 30 – 80 m           |
| `flight_type`| Different heuristic               | 0, 1, 2             |

```bash
# Iterate until green
bun test:run edge_no_path --max-vel 0.3
bun test:run edge_no_path --max-vel 0.6
bun test:run edge_no_path --max-vel 1.0
```

**Green verification (via API)**:

```bash
# Check success rate
curl -s http://localhost:3000/api/test/edge_no_path-1 \
  | jq '.plan_result | map(select(.plan_status == false)) | length'
```

Expected: `0`.

**Visual check**: open `bun dashboard` and confirm:
- 3D trajectory shows continuous movement covering map area
- StatsGrid shows `success: 100%`

---

## Phase 3: Refactor — Optimize the Test Harness

### 3a. Parallelization

```bash
# Increase parallel containers in config.ts
maxContainers: 8  →  maxContainers: 16
```

Monitor GPU/memory pressure:

```bash
nvidia-smi
docker stats --no-stream
```

If containers stall (Xvfb + OpenGL contention), reduce `--gpus` per container or batch them manually.

### 3b. Comparison of Multiple Runs

Once multiple `_site/test-results/<scenario>-*/` directories exist, compare:

```bash
# Sample counts per run
for d in _site/test-results/velocity_sweep-*/; do
  id=$(basename "$d")
  samples=$(cat "$d/odom.jsonl" 2>/dev/null | wc -l)
  plans=$(cat "$d/plan_result.jsonl" 2>/dev/null | wc -l)
  fails=$(grep -c '"plan_status":false' "$d/plan_result.jsonl" 2>/dev/null || echo 0)
  echo "$id  samples=$samples  plans=$plans  fails=$fails"
done
```

### 3c. Structured Summary Report

The test harness should produce a machine-readable summary per run:

```bash
# Write to _site/test-results/<id>/summary.json
# Server auto-generates this on container exit
curl http://localhost:3000/api/tests
```

Expected summary:

```json
{
  "id": "velocity_sweep-0_6",
  "active": false,
  "samples": 1520,
  "plans": 340,
  "fails": 0,
  "duration": 300
}
```

### 3d. CLI Report Command

```bash
bun test:report
# Prints a table:
# ┌──────────────────────────┬────────┬────────┬────────┬──────────┐
# │ id                       │ samples│  plans │  fails │ duration │
# ├──────────────────────────┼────────┼────────┼────────┼──────────┤
# │ velocity_sweep-0_3       │   1080 │    240 │      0 │ 300      │
# │ velocity_sweep-0_6       │   1520 │    340 │      0 │ 300      │
# │ velocity_sweep-1         │   1100 │    280 │     12 │ 300      │
# └──────────────────────────┴────────┴────────┴────────┴──────────┘
```

---

## Common Failure Patterns

| Symptom                                        | Likely cause                        | Fix                                    |
| ---------------------------------------------- | ----------------------------------- | -------------------------------------- |
| Container exits in <5s                         | `sed` path bug in entrypoint        | Check `docker logs <name>`             |
| Container stays `running` but no MQTT data     | MQTT broker not running             | `sudo systemctl start mosquitto`       |
| Container runs full duration but data empty    | Bun server not running during test  | Start `bun server` before `bun test:run` |
| Frontend shows no historical data              | Test ran before server was started   | Restart server; data from old runs is lost |
| Container immediately crashes with exit 1      | GPU out of memory / OpenGL context   | Reduce `maxContainers`                 |

---

## TDD Cycle Checklist

- [ ] Red: scenario produces at least one `plan_status=false`
- [ ] Green: same scenario after param tweak produces 0 `plan_status=false`
- [ ] Refactor: at least 3 runs compared, summary table generated
- [ ] ADR updated if new infrastructure decision (e.g., new data format)
