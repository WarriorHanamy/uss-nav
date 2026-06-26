import { readFileSync, existsSync, readdirSync } from "fs";
import { join } from "path";
import { CFG } from "../core/config";
import type { ValidationReport } from "../types/ego-test";
import { buildValidationReport } from "./test-assert";

const RESULT_DIR = join(import.meta.dir, "../..", CFG.testResultDir);

function loadTestData(testId: string): Record<string, unknown[]> {
  const dir = join(RESULT_DIR, testId);
  if (!existsSync(dir)) {
    console.error(`[validate] No data for test: ${testId}`);
    process.exit(1);
  }

  const data: Record<string, unknown[]> = {};
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
  }
  return data;
}

function printReport(report: ValidationReport): void {
  console.log(`\n=== Validation Report: ${report.testId} ===`);
  console.log(`\nSample Counts:`);
  for (const [topic, count] of Object.entries(report.sampleCounts)) {
    console.log(`  ${topic}: ${count}`);
  }

  console.log(`\nOdometry:`);
  console.log(`  total samples: ${report.odometry.total}`);
  console.log(`  in bounds:     ${report.odometry.inBounds}`);
  console.log(`  max velocity:  ${report.odometry.maxVelocity.toFixed(3)} m/s`);
  console.log(`  over limit:    ${report.odometry.maxVelocityExceeded}`);
  console.log(`  min height:    ${report.odometry.minHeight.toFixed(3)} m`);

  console.log(`\nPlan Results:`);
  console.log(`  total:         ${report.planResults.total}`);
  console.log(`  success count: ${report.planResults.successCount}`);
  console.log(`  success rate:  ${(report.planResults.successRate * 100).toFixed(1)}%`);
  console.log(`  avg plan time: ${report.planResults.avgPlanTimes.toFixed(2)}`);

  console.log(`\nState Triggers:`);
  console.log(`  total:         ${report.stateTriggers.total}`);
  console.log(`  triggered:     ${report.stateTriggers.triggeredCount}`);

  console.log(`\nExec Finishes:`);
  console.log(`  total:         ${report.execFinishes.total}`);
  console.log(`  finished:      ${report.execFinishes.finishedCount}`);
}

export async function cmdTestValidate(
  testId: string,
  maxX: number = 50,
  maxY: number = 30,
  maxZ: number = 5,
  maxVel: number = 0.7,
): Promise<ValidationReport> {
  const data = loadTestData(testId);
  const report = buildValidationReport(testId, data, maxX, maxY, maxZ, maxVel);
  printReport(report);

  const healthy =
    report.odometry.total > 0 &&
    report.odometry.inBounds &&
    !report.odometry.maxVelocityExceeded &&
    report.planResults.successRate > 0.3;

  console.log(`\nOverall: ${healthy ? "HEALTHY" : "UNHEALTHY"}`);
  return report;
}

if (import.meta.main) {
  const testId = process.argv[2];
  if (!testId) {
    console.error("Usage: bun test:validate <test-id>");
    process.exit(1);
  }
  cmdTestValidate(testId).then((r) => process.exit(0));
}
