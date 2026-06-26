# Skill: ego-test-tdd

# Docker-based TDD with Host Post-Processing

A three-phase TDD cycle for codebases where system-under-test runs in Docker
containers while test orchestration and assertions execute on the development
host. This pattern handles the unavoidable cross-language data pipeline between
container-internal code (C++, Python, ROS, etc.) and host-side test logic
(TypeScript, Python, Go, etc.).

```
Red       Write a machine-verifiable test for behavior that doesn't exist yet
Green     Modify container code, build image, run test, observe pass
Refactor  Improve structure of system code, test harness, or data contract
```

---

## Architecture Prerequisites

This workflow assumes the entity topology defined in
[dev-entity-ids](../dev-entity-ids/SKILL.md):

```
devel-host  ──docker build──>  test-image
                                   │
                               docker run
                                   │
                                   ▼
                             test-container  (system-under-test)
                                   │
                  ──data transport──>  devel-host
                                           │
                                     post-processing
                                           │
                                     assertions
```

### Data Flow Direction

```
test-container (internal code)  ──>  transport layer  ──>  devel-host
                                                                 │
                                                       collect + transform
                                                                 │
                                                       machine-verifiable
                                                       assertions
```

Key constraints (from [docker-dev-mounts](../docker-dev-mounts/SKILL.md) and
[docker-image-naming](../docker-image-naming/SKILL.md)):

| Principle | Rationale |
|-----------|-----------|
| No bind mounts | Image is self-contained; parameterization via env vars |
| Immutable per run | Every run starts from same image; no side effects |
| Data flows outward only | Container never reads from host; no MQTT/HTTP back-channel |
| Ephemeral container | Container exits after test duration; all data persisted on host |
| Image naming convention | `<role>:<variant>` — explicit tags, no implicit `latest` for CI |

---

## Cross-Language Data Contract

The central complexity in this architecture: test assertions on devel-host
consume data produced by code in a different language running in the container.
The **schema is the contract** between these two worlds.

### Schema-First Design

1. Define the data schema once (JSON Schema, Protobuf `.proto`, or TypeScript type)
2. Generate type bindings for both sides:
   - Container side: Python `dataclass` / C++ struct with serialization
   - Host side: TypeScript interface / Python `dataclass`
3. Check generated types into version control beside the schema definition

### Common Cross-Language Traps

| Issue | Language A (host) | Language B (container) | Symptom |
|-------|-------------------|------------------------|---------|
| Field naming | `camelCase` (TS/JS) | `snake_case` (Python/C++) | Fields silently missing from parsed objects |
| Numeric precision | `number` (float64) | `float32` (C++ ROS msg) | Trailing decimals truncated on host side |
| Enum values | Starts at `0` (TS enum) | Starts at `1` (Protobuf) | Off-by-one logic errors |
| Timestamps | `Date.now()` ms epoch | `rostime` sec+nsec | Wrong time scaling on host |
| Optional fields | `field?: T` | `field` default-initialized | Host expects `undefined`, gets zero value |
| String encoding | UTF-8 (JS default) | ASCII C-string | Non-ASCII chars cause parse failures |

### Transport Reliability

| Transport | Typical use | Failure mode | Mitigation |
|-----------|-------------|--------------|------------|
| MQTT | Real-time telemetry | QoS 0 drops on broker restart | Use QoS 1; verify `mosquitto_sub -t 'test/#' -v` |
| WebSocket | Live dashboard | Reconnect gap loses intermediate messages | Buffer on server; replay from file on reconnect |
| File export | Post-hoc analysis | Container crashes before flush | Periodic fsync inside container |
| HTTP API | Command/response | Timeout on slow computation | Set explicit timeout; separate sync vs async endpoints |

### Debugging Contract Mismatches

When an assertion fails unexpectedly, dump raw payload at both ends:

```bash
# On devel-host: log raw incoming messages before parsing
mosquitto_sub -t 'test/#' -v > /tmp/raw_mqtt_dump.log

# Inside container: log what was serialized before sending
docker exec <container> tail -f /tmp/sent_payloads.jsonl
```

Compare field-by-field. If the host parser drops a field, it's almost always a
naming or type mismatch.

---

## Phase 1: Red — Write a Failing Test

A Red-phase test describes **what the system should do**, not how it should do
it. The test must fail because the behavior does not yet exist — not because of
infrastructure problems.

### Anatomy of a Red Test

```text
Given:  system is initialized with known configuration
When:   specific stimulus is applied (or condition is met)
Then:   observable output meets expected criteria
```

The "observable output" must be machine-verifiable: a JSON field, a file
content, a status code. Not a human reading logs.

### Infrastructure Failure vs Behavioral Failure

| Signal | Likely infrastructure failure | Likely behavioral failure |
|--------|-------------------------------|---------------------------|
| Container exits before test duration | Entrypoint crash, missing dependency | Code throws on valid input (still a bug, but subtle) |
| No data arrives at host | MQTT broker down, network config | Code never publishes (assertion timeout) |
| Data arrives but all fields are default | Schema mismatch or empty source | Code returns degenerate result |
| Container runs full duration, data looks plausible | False green — check threshholds | True behavior exhibited |

### Red Phase Checklist

- [ ] Test exercises a public interface (MQTT topic, HTTP endpoint, exported file)
- [ ] Assertion is machine-verifiable (numeric comparison, JSON path match, count)
- [ ] Failure is reproducible: `bun test:run <id>` produces same red signal every time
- [ ] Red signal is distinguishable from infrastructure problems
- [ ] No test logic depends on container-internal state (only on published data)
- [ ] The test is the minimum needed to express the missing behavior

### Common Mistakes

| Mistake | Why it's wrong | Fix |
|---------|----------------|-----|
| Testing implementation (private function, internal ROS topic) | Test breaks on refactor that doesn't change behavior | Test only through public data stream |
| Writing multiple tests before implementing | Horizontal slicing → tests test imagined behavior | One test, then implement, then repeat |
| Asserting on human-readable output | Can't automate | Assert on machine fields, then view logs manually for context |
| Setting impossible thresholds | Flaky tests | Base threshold on empirical observation of working system |

---

## Phase 2: Green — Minimal Implementation

Write or modify code inside the container to make the single Red test pass.
Build a new image, run, collect data, assert.

### Minimal Implementation Rules

1. **One test at a time** — vertical slice, not horizontal batch
2. **Minimum code to pass** — no speculative generalization, no unused branches
3. **Build once per cycle** — hot-reload is not available inside container;
   rebuild image with `docker build` or equivalent
4. **Verify pass signal is unambiguous** — the assertion that failed in Red
   now produces a clear pass; any new failure means the implementation is
   incomplete or the test was wrong

### Green Verification Flow

```bash
# 1. Build image with modified code
bun test:build

# 2. Run container with test scenario
docker run -d \
  --name test-<id> \
  -e TEST_ID=<id> \
  ... other env vars ...
  test-image:<variant>

# 3. Wait for data or timeout
bun test:wait <id> --timeout 120

# 4. Run host-side assertion
bun test:assert <id> --expect '<json-path> == <expected>'
```

If step 4 returns zero (pass), Green is achieved. If it returns non-zero,
the implementation is incomplete — iterate on container code only.

### What NOT to Do in Green

- Do not generalize before the next test forces it (YAGNI)
- Do not add tests while implementing (stay focused on making one test pass)
- Do not refactor system code (that's Phase 3)
- Do not tune transport parameters to make a flaky test pass (fix the test)

---

## Phase 3: Refactor — Improve Without Changing Behavior

With all tests Green, improve the code structure, test harness, or data
contract. Tests must remain Green after each refactor step.

### Categories of Refactoring

#### 1. System-Under-Test Code (inside container)

- Extract duplicated logic in Python bridge or C++ nodes
- Deepen modules: hide complex computation behind simple interfaces
- Consolidate ROS message types or Python dataclasses
- Improve error propagation (clearer exceptions → cleaner test failures)

#### 2. Test Harness Code (on devel-host)

- Reduce latency between container exit and assertion completion
- Add retry logic for transient transport failures (with explicit upper bound)
- Improve assertion error messages (show expected vs actual diff)
- Parallelize independent test runs (but keep per-run assertions atomic)

#### 3. Data Contract Evolution

- Add new fields to schema (backward compatible: new field is optional)
- Deprecate old fields with a migration window
- Rename fields only if both ends can be deployed atomically
- Update generated type bindings and check schema version into VCS

#### 4. Pipeline Optimization

| Bottleneck | Diagnosis | Fix |
|------------|-----------|-----|
| Image build dominates cycle | `time docker build` shows long layer rebuild | Layer cache ordering, split base image |
| Transport congested | MQTT message rate > broker throughput | Batch messages, reduce publish rate, or use WebSocket endpoint |
| Host post-processing late | `bun test:assert` runs after data file written | Stream-parse data instead of reading whole file at end |

### Refactor Checklist

- [ ] All tests Green before starting any refactor
- [ ] One refactor per cycle (not a rewrite marathon)
- [ ] Tests pass after each refactor commit
- [ ] ADR updated if infrastructure decision changed (image structure, transport, schema)
- [ ] No behavior change — output data is identical before and after (diff the `.jsonl`)

---

## End-to-End TDD Cycle Checklist

```
[ ] Red:   Test describes absent behavior, fails with machine-verifiable signal
[ ]        Infrastructure failure distinguished from behavioral failure
[ ]        Only one test written
[ ] Green: Container code modified minimally, build → run → collect → pass
[ ]        New image built, no speculative features
[ ]        Pass signal unambiguous
[ ] Refactor: System code, test harness, or data contract improved
[ ]          All tests remain Green
[ ]          ADR if infrastructure changed
```

## References

| Skill | Role |
|-------|------|
| [dev-entity-ids](../dev-entity-ids/SKILL.md) | Entity topology, host vs container roles, data pipeline chains |
| [docker-dev-mounts](../docker-dev-mounts/SKILL.md) | Container immutability, env-var parameterization, no bind mounts |
| [docker-image-naming](../docker-image-naming/SKILL.md) | Image tag convention, build commands |
| [tdd](../../../.config/opencode/skills/tdd/SKILL.md) | General TDD philosophy, vertical slice, public interface principle |
