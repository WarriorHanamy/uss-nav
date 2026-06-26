import { readFileSync, existsSync, readdirSync, mkdirSync, appendFileSync } from "fs";
import { join } from "path";
import { CFG } from "../core/config";
import type { SmokeTestReport, Assertion } from "../types/ego-test";
import {
  assertNonEmpty,
  assertOdometryBounds,
  assertMaxVelocity,
  assertPlanSuccessRate,
  assertStateTrigger,
  assertExecFinish,
  assertDataFlow,
} from "./test-assert";

const RESULT_DIR = join(import.meta.dir, "../..", CFG.testResultDir);

function runAssertions(
  data: Record<string, unknown[]>,
  maxX: number,
  maxY: number,
  maxZ: number,
  maxVel: number,
): Assertion[] {
  const all: Assertion[] = [];

  all.push(...assertDataFlow(data, ["odom", "plan_result", "state_trigger", "exec_finish", "data_disp"]));

  const odom = (data["odom"] || []) as import("../types/ego-test").OdometrySample[];
  const planResults = (data["plan_result"] || []) as import("../types/ego-test").PlanResultSample[];
  const stateTriggers = (data["state_trigger"] || []) as import("../types/ego-test").StateTriggerSample[];
  const execFinishes = (data["exec_finish"] || []) as import("../types/ego-test").ExecFinishSample[];

  all.push(assertNonEmpty(odom, "odom.nonempty"));
  all.push(assertNonEmpty(planResults, "plan_result.nonempty"));

  all.push(...assertOdometryBounds(odom, maxX, maxY, maxZ));
  all.push(...assertMaxVelocity(odom, maxVel));
  all.push(...assertPlanSuccessRate(planResults, 0.3));
  all.push(...assertStateTrigger(stateTriggers));
  all.push(...assertExecFinish(execFinishes));

  return all;
}

async function waitForData(
  testId: string,
  timeoutMs: number,
  pollMs: number = 2000,
): Promise<Record<string, unknown[]> | null> {
  const dir = join(RESULT_DIR, testId);
  const start = Date.now();

  while (Date.now() - start < timeoutMs) {
    if (!existsSync(dir)) {
      await new Promise((r) => setTimeout(r, pollMs));
      continue;
    }

    const data: Record<string, unknown[]> = {};
    let hasAny = false;

    for (const file of readdirSync(dir)) {
      if (!file.endsWith(".jsonl")) continue;
      const subtopic = file.replace(".jsonl", "");
      const content = readFileSync(join(dir, file), "utf-8").trim();
      if (!content) continue;
      data[subtopic] = content.split("\n")
        .filter(Boolean)
        .map((line) => {
          try { return JSON.parse(line); } catch { return null; }
        })
        .filter(Boolean);
      if (data[subtopic].length > 0) hasAny = true;
    }

    if (hasAny) {
      const odom = data["odom"] || [];
      const planResult = data["plan_result"] || [];
      if (odom.length >= 10 && planResult.length >= 1) {
        return data;
      }
    }

    await new Promise((r) => setTimeout(r, pollMs));
  }

  return null;
}

export async function cmdTestSmoke(
  testId?: string,
  duration?: number,
  maxVel?: number,
  obsNum?: number,
): Promise<SmokeTestReport> {
  const id = testId || `smoke-${Date.now()}`;
  const dur = duration ?? 60;
  const vel = maxVel ?? 0.6;
  const obs = obsNum ?? 30;
  const startTime = Date.now();

  console.log(`[smoke] Test ID: ${id}`);
  console.log(`[smoke] Duration: ${dur}s, max_vel: ${vel}, obs_num: ${obs}`);

  const containerName = `ego-test-${id}`;
  const env = [
    "-e", `TEST_ID=${id}`,
    "-e", "MQTT_HOST=localhost",
    "-e", `DURATION=${dur}`,
    "-e", `MAX_VEL=${vel}`,
    "-e", `MAX_ACC=1.0`,
    "-e", `OBS_NUM=${obs}`,
    "-e", `FLIGHT_TYPE=2`,
    "-e", `X_SIZE=50`,
    "-e", `Y_SIZE=30`,
  ];

  const args = [
    "run", "-d", "--rm",
    "--name", containerName,
    "--network", "host",
    "--gpus", "all",
    "--ipc=host",
    "--security-opt", "seccomp=unconfined",
    ...env,
    CFG.dockerImage,
  ];

  console.log(`[smoke] Starting container ${containerName}...`);
  const proc = Bun.spawn(["docker", ...args], {
    stdout: "inherit",
    stderr: "inherit",
  });
  const exitCode = await proc.exited;
  if (exitCode !== 0) {
    console.error(`[smoke] Failed to start container`);
    const report: SmokeTestReport = {
      testId: id,
      duration: 0,
      startTime,
      endTime: Date.now(),
      assertions: [{ name: "docker.start", status: "error", message: "docker run failed" }],
      passed: 0, failed: 0, errors: 1, overall: "error",
    };
    return report;
  }
  console.log(`[smoke] Container started`);

  const waitSec = Math.min(dur + 20, 120);
  console.log(`[smoke] Waiting up to ${waitSec}s for data...`);
  const data = await waitForData(id, waitSec * 1000, 3000);

  // Stop container
  console.log(`[smoke] Stopping container...`);
  await Bun.spawn(["docker", "stop", containerName, "--time", "5"]).exited;

  const endTime = Date.now();

  if (!data) {
    const report: SmokeTestReport = {
      testId: id,
      duration: Math.round((endTime - startTime) / 1000),
      startTime,
      endTime,
      assertions: [{
        name: "data_flow.overall",
        status: "fail",
        message: "timed out waiting for data",
      }],
      passed: 0,
      failed: 1,
      errors: 0,
      overall: "fail",
    };
    console.log(`[smoke] FAIL: no data received within ${waitSec}s`);
    return report;
  }

  const assertions = runAssertions(data, 50, 30, 5, vel);
  const passed = assertions.filter((a) => a.status === "pass").length;
  const failed = assertions.filter((a) => a.status === "fail").length;
  const errors = assertions.filter((a) => a.status === "error").length;
  const overall = failed === 0 && errors === 0 ? "pass" : "fail";

  const report: SmokeTestReport = {
    testId: id,
    duration: Math.round((endTime - startTime) / 1000),
    startTime,
    endTime,
    assertions,
    passed,
    failed,
    errors,
    overall,
  };

  const reportPath = join(RESULT_DIR, id, "_smoke_report.json");
  mkdirSync(join(RESULT_DIR, id), { recursive: true });
  appendFileSync(reportPath, JSON.stringify(report, null, 2) + "\n");

  console.log(`\n[smoke] === Smoke Test Report: ${id} ===`);
  for (const a of assertions) {
    const icon = a.status === "pass" ? "✓" : a.status === "fail" ? "✗" : "!";
    console.log(`  ${icon} ${a.name}: ${a.message}`);
    if (a.status === "fail" && a.expected) {
      console.log(`      expected: ${a.expected}`);
      console.log(`      actual:   ${a.actual}`);
    }
  }
  console.log(`\n[smoke] Passed: ${passed}, Failed: ${failed}, Errors: ${errors}`);
  console.log(`[smoke] Overall: ${overall === "pass" ? "PASS" : "FAIL"}`);

  return report;
}

if (import.meta.main) {
  const id = process.argv[2];
  const dur = process.argv[3] ? parseInt(process.argv[3]) : 60;
  const vel = process.argv[4] ? parseFloat(process.argv[4]) : 0.6;
  cmdTestSmoke(id, dur, vel).then((r) => process.exit(r.overall === "pass" ? 0 : 1));
}
