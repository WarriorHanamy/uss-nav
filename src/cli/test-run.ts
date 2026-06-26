import { CFG } from "../core/config";
import { BUILTIN_SCENARIOS, expandScenario } from "./scenarios";
import { cmdTestStop } from "./test-stop";
import type { TestConfig } from "../types/ego-test";

async function dockerRunTest(cfg: TestConfig): Promise<void> {
  const containerName = `ego-test-${cfg.id}`;
  const env: Record<string, string> = {
    TEST_ID: cfg.id,
    MQTT_HOST: "localhost",
    FLIGHT_TYPE: String(cfg.params.flight_type ?? 2),
    MAX_VEL: String(cfg.params.max_vel ?? 0.6),
    MAX_ACC: String(cfg.params.max_acc ?? 1.0),
    OBS_NUM: String(cfg.params.obs_num ?? 30),
    X_SIZE: String(cfg.params.x_size ?? 50),
    Y_SIZE: String(cfg.params.y_size ?? 30),
    DURATION: String(cfg.duration),
  };

  const envArgs = Object.entries(env).flatMap(([k, v]) => ["-e", `${k}=${v}`]);
  const args = [
    "run", "-d", "--rm",
    "--name", containerName,
    "--network", "host",
    "--gpus", "all",
    "--ipc=host",
    "--security-opt", "seccomp=unconfined",
    ...envArgs,
    CFG.dockerImage,
  ];

  const proc = Bun.spawn(["docker", ...args], {
    stdin: "inherit",
    stdout: "inherit",
    stderr: "inherit",
  });

  const exitCode = await proc.exited;
  if (exitCode === 0) {
    console.log(`  ✓ ${containerName}`);
  } else {
    console.error(`  ✗ ${containerName} failed`);
  }
}

export async function cmdTestRun(scenarioId?: string): Promise<void> {
  const scenarios = scenarioId
    ? BUILTIN_SCENARIOS.filter((s) => s.id === scenarioId)
    : BUILTIN_SCENARIOS;

  if (scenarios.length === 0) {
    console.error(`[test-run] No scenario: ${scenarioId}`);
    process.exit(1);
  }

  // auto-reset: stop all previous test containers
  console.log("[test-run] Clearing previous runs...");
  await cmdTestStop();

  let total = 0;

  for (const scenario of scenarios) {
    const configs = expandScenario(scenario);
    console.log(`[test-run] Scenario "${scenario.id}": ${configs.length} config(s)`);

    for (let i = 0; i < configs.length; i += CFG.maxContainers) {
      const batch = configs.slice(i, i + CFG.maxContainers);
      console.log(`  batch ${Math.floor(i / CFG.maxContainers) + 1}: ${batch.length} container(s)`);

      await Promise.all(batch.map(dockerRunTest));
      total += batch.length;
    }
  }

  console.log(`[test-run] ${total} container(s) started`);
}
