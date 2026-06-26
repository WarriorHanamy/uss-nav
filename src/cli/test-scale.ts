import { readFileSync, readdirSync, existsSync } from "fs";
import { join } from "path";
import { CFG } from "../core/config";
import type { ScaleTestConfig, ScaleTestReport } from "../types/ego-test";

const RESULT_DIR = join(import.meta.dir, "../..", CFG.testResultDir);

interface ContainerInfo {
  id: string;
  status: string;
  sampleCount: number;
}

async function launchContainer(
  testId: string,
  params: Record<string, number>,
  duration: number,
): Promise<boolean> {
  const containerName = `ego-test-${testId}`;
  const env = [
    "-e", `TEST_ID=${testId}`,
    "-e", `MQTT_HOST=host.docker.internal`,
    "-e", `DURATION=${duration}`,
    "-e", `MAX_VEL=${params.max_vel ?? 0.6}`,
    "-e", `MAX_ACC=${params.max_acc ?? 1.0}`,
    "-e", `OBS_NUM=${params.obs_num ?? 30}`,
    "-e", `FLIGHT_TYPE=${params.flight_type ?? 2}`,
    "-e", `X_SIZE=${params.x_size ?? 50}`,
    "-e", `Y_SIZE=${params.y_size ?? 30}`,
  ];

  const args = [
    "run", "-d", "--rm",
    "--name", containerName,
    "--gpus", "all",
    "--ipc=host",
    "--security-opt", "seccomp=unconfined",
    "--add-host", "host.docker.internal:host-gateway",
    ...env,
    CFG.dockerImage,
  ];

  const proc = Bun.spawn(["docker", ...args], {
    stdout: "pipe",
    stderr: "pipe",
  });
  const exitCode = await proc.exited;
  return exitCode === 0;
}

async function getDockerStats(): Promise<{ cpu: number; mem: number }> {
  try {
    const proc = Bun.spawn([
      "docker", "stats", "--no-stream",
      "--format", "{{.CPUPerc}} {{.MemPerc}}",
    ], { stdout: "pipe" });
    const output = await new Response(proc.stdout).text();
    const lines = output.trim().split("\n");
    let totalCpu = 0;
    let totalMem = 0;
    let count = 0;
    for (const line of lines) {
      const parts = line.split(" ");
      if (parts.length >= 2) {
        const cpu = parseFloat(parts[0].replace("%", ""));
        const mem = parseFloat(parts[1].replace("%", ""));
        if (!isNaN(cpu) && !isNaN(mem)) {
          totalCpu += cpu;
          totalMem += mem;
          count++;
        }
      }
    }
    return { cpu: count > 0 ? totalCpu / count : 0, mem: count > 0 ? totalMem / count : 0 };
  } catch {
    return { cpu: 0, mem: 0 };
  }
}

function countSamples(testId: string): number {
  const dir = join(RESULT_DIR, testId);
  if (!existsSync(dir)) return 0;
  let total = 0;
  for (const file of readdirSync(dir)) {
    if (!file.endsWith(".jsonl")) continue;
    const content = readFileSync(join(dir, file), "utf-8").trim();
    if (content) total += content.split("\n").length;
  }
  return total;
}

export async function cmdTestScale(
  count: number = 4,
  duration: number = 120,
  batchSize: number = CFG.maxContainers,
): Promise<ScaleTestReport> {
  const config: ScaleTestConfig = {
    count,
    params: { max_vel: 0.6, max_acc: 1.0, obs_num: 30, flight_type: 2, x_size: 50, y_size: 30 },
    duration,
    batchSize,
    containerPrefix: `scale-${Date.now()}`,
  };

  console.log(`[scale] Starting ${count} containers in batches of ${batchSize}`);
  console.log(`[scale] Duration: ${duration}s each`);
  console.log(`[scale] Params: max_vel=${config.params.max_vel}, obs_num=${config.params.obs_num}`);

  const startTime = Date.now();
  const containers: ContainerInfo[] = [];
  let launched = 0;
  let succeeded = 0;
  let failed = 0;

  // Resource monitoring
  const cpuSamples: number[] = [];
  const memSamples: number[] = [];

  const monitorInterval = setInterval(async () => {
    const stats = await getDockerStats();
    cpuSamples.push(stats.cpu);
    memSamples.push(stats.mem);
  }, 5000);

  while (launched < count) {
    const batchSize = Math.min(config.batchSize, count - launched);
    const promises: Promise<boolean>[] = [];

    for (let i = 0; i < batchSize; i++) {
      const idx = launched + i;
      const testId = `${config.containerPrefix}-${idx}`;
      promises.push(launchContainer(testId, config.params, config.duration));
    }

    const results = await Promise.all(promises);
    for (let i = 0; i < results.length; i++) {
      const idx = launched + i;
      const testId = `${config.containerPrefix}-${idx}`;
      if (results[i]) {
        succeeded++;
        containers.push({ id: testId, status: "started", sampleCount: 0 });
        console.log(`  [scale] ✓ ${testId}`);
      } else {
        failed++;
        containers.push({ id: testId, status: "failed", sampleCount: 0 });
        console.log(`  [scale] ✗ ${testId} — launch failed`);
      }
    }

    launched += batchSize;

    if (launched < count) {
      console.log(`  [scale] waiting for batch to settle...`);
      await new Promise((r) => setTimeout(r, 5000));
    }
  }

  // Wait for all containers to finish
  const totalWait = duration + 30;
  console.log(`[scale] Waiting ${totalWait}s for all containers to complete...`);
  await new Promise((r) => setTimeout(r, totalWait * 1000));

  clearInterval(monitorInterval);

  // Collect results
  for (const c of containers) {
    c.sampleCount = countSamples(c.id);
    c.status = c.sampleCount > 0 ? "completed" : "no_data";
  }

  const endTime = Date.now();
  const avgCpu = cpuSamples.length > 0
    ? cpuSamples.reduce((s, v) => s + v, 0) / cpuSamples.length : 0;
  const peakCpu = cpuSamples.length > 0 ? Math.max(...cpuSamples) : 0;
  const avgMem = memSamples.length > 0
    ? memSamples.reduce((s, v) => s + v, 0) / memSamples.length : 0;
  const peakMem = memSamples.length > 0 ? Math.max(...memSamples) : 0;

  const report: ScaleTestReport = {
    config,
    startTime,
    endTime,
    totalDuration: Math.round((endTime - startTime) / 1000),
    containersLaunched: launched,
    containersSucceeded: succeeded,
    containersFailed: failed,
    containers,
    resourceUsage: { avgCpu, peakCpu, avgMem, peakMem },
  };

  console.log(`\n[scale] === Scale Test Report ===`);
  console.log(`  Containers: ${containers.length} total`);
  console.log(`  Succeeded:  ${report.containersSucceeded}`);
  console.log(`  Failed:     ${report.containersFailed}`);
  console.log(`  Duration:   ${report.totalDuration}s`);

  const withData = containers.filter((c) => c.sampleCount > 0).length;
  console.log(`  With data:  ${withData}/${containers.length}`);

  console.log(`\n  Resource Usage (avg):`);
  console.log(`    CPU:  ${avgCpu.toFixed(1)}% (peak ${peakCpu.toFixed(1)}%)`);
  console.log(`    Mem:  ${avgMem.toFixed(1)}% (peak ${peakMem.toFixed(1)}%)`);

  if (containers.length > 0) {
    const totalSamples = containers.reduce((s, c) => s + c.sampleCount, 0);
    const avgPerContainer = totalSamples / containers.length;
    console.log(`\n  Data:`);
    console.log(`    Total samples:  ${totalSamples}`);
    console.log(`    Avg/container:  ${avgPerContainer.toFixed(0)}`);
  }

  return report;
}

if (import.meta.main) {
  const count = process.argv[2] ? parseInt(process.argv[2]) : 4;
  const duration = process.argv[3] ? parseInt(process.argv[3]) : 120;
  const batchSize = process.argv[4] ? parseInt(process.argv[4]) : CFG.maxContainers;
  cmdTestScale(count, duration, batchSize).then(() => process.exit(0));
}
