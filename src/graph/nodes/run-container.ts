import { CFG } from "../../core/config";
import { extractAllMetrics } from "../../core/telemetry";

function buildEnv(config: any): Record<string, string> {
  return {
    TEST_ID: config.id,
    MQTT_HOST: "host.docker.internal",
    FLIGHT_TYPE: String(config.params.flight_type ?? 2),
    MAX_VEL: String(config.params.max_vel ?? 0.6),
    MAX_ACC: String(config.params.max_acc ?? 1.0),
    OBS_NUM: String(config.params.obs_num ?? 30),
    X_SIZE: String(config.params.x_size ?? 50),
    Y_SIZE: String(config.params.y_size ?? 30),
    DURATION: String(config.duration ?? CFG.defaultDuration),
  };
}

export async function runContainer(state: Record<string, any>): Promise<Record<string, any>> {
  const config = state.config;
  if (!config || !config.id) {
    return { config: null };
  }

  const safeId = config.id.replace(/[=_\.\s]/g, "-");
  const name = `uss-nav-test-${safeId}`;
  const env = buildEnv(config);
  const envArgs = Object.entries(env).flatMap(([k, v]) => ["-e", `${k}=${v}`]);

  console.log(`  [${config.id}] starting...`);

  const proc = Bun.spawn(
    ["docker", "compose", "run", "--rm", "--name", name, ...envArgs, "test"],
    { stdout: "inherit", stderr: "inherit" }
  );

  const exitCode = await proc.exited;
  console.log(`  [${config.id}] exit=${exitCode}`);

  const metrics = await extractAllMetrics(config.id);

  return {
    results: [{ testId: config.id, exitCode, metrics }],
    config: null,
  };
}
