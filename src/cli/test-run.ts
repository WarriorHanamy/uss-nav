import { extractAllMetrics } from "../core/telemetry";
import { CFG } from "../core/config";

function buildEnv(config: Record<string, any>): Record<string, string> {
  const p = config.params ?? config;
  return {
    TEST_ID: config.id ?? "test",
    MQTT_HOST: "host.docker.internal",
    FLIGHT_TYPE: String(p.flight_type ?? 2),
    MAX_VEL: String(p.max_vel ?? 0.6),
    MAX_ACC: String(p.max_acc ?? 1.0),
    OBS_NUM: String(p.obs_num ?? 30),
    X_SIZE: String(p.x_size ?? 50),
    Y_SIZE: String(p.y_size ?? 30),
    DURATION: String(config.duration ?? CFG.defaultDuration),
  };
}

export async function cmdTestRun(config: Record<string, any>): Promise<{ testId: string; exitCode: number; metrics: Record<string, number> }> {
  const name = `uss-nav-test-${(config.id ?? "unknown").replace(/[._]/g, "-")}`;
  const env = buildEnv(config);
  const envArgs = Object.entries(env).flatMap(([k, v]) => ["-e", `${k}=${v}`]);

  console.log(`  [test-run] starting ${name}...`);

  const proc = Bun.spawn(
    ["docker", "compose", "run", "--rm", "--name", name, ...envArgs, "test"],
    { stdout: "inherit", stderr: "inherit", cwd: process.env.PWD ?? process.cwd() }
  );

  const exitCode = await proc.exited;
  const testId = config.id ?? "test";
  const metrics = await extractAllMetrics(testId);

  console.log(`  [test-run] ${testId} exit=${exitCode}`);

  return { testId, exitCode, metrics };
}
