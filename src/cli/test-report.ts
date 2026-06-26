import { readdirSync, existsSync, readFileSync, statSync } from "fs";
import { join } from "path";
import { CFG } from "../core/config";
import { buildValidationReport } from "./test-assert";

const RESULT_DIR = join(import.meta.dir, "../..", CFG.testResultDir);

export async function cmdTestReport(): Promise<void> {
  if (!existsSync(RESULT_DIR)) {
    console.log(`[report] No test results directory: ${RESULT_DIR}`);
    return;
  }

  const dirs = readdirSync(RESULT_DIR)
    .filter((name) => {
      const p = join(RESULT_DIR, name);
      try { return statSync(p).isDirectory(); } catch { return false; }
    })
    .sort()
    .reverse();

  if (dirs.length === 0) {
    console.log(`[report] No test runs found. Run 'bun test:run' first.`);
    return;
  }

  console.log(`\n=== Test Report Summary ===`);
  console.log(`Found ${dirs.length} test run(s)\n`);

  for (const dir of dirs) {
    const data: Record<string, unknown[]> = {};
    const dirPath = join(RESULT_DIR, dir);
    let hasData = false;

    for (const file of readdirSync(dirPath)) {
      if (!file.endsWith(".jsonl")) continue;
      const subtopic = file.replace(".jsonl", "");
      const content = readFileSync(join(dirPath, file), "utf-8").trim();
      if (!content) continue;
      data[subtopic] = content.split("\n")
        .filter(Boolean)
        .map((line) => {
          try { return JSON.parse(line); } catch { return null; }
        })
        .filter(Boolean);
      if (data[subtopic].length > 0) hasData = true;
    }

    if (!hasData) {
      console.log(`  ${dir}: <no data>`);
      continue;
    }

    const report = buildValidationReport(dir, data, 50, 30, 5, 0.7);
    const health = report.odometry.total > 0 && report.odometry.inBounds && !report.odometry.maxVelocityExceeded
      ? "HEALTHY" : "ISSUES";
    console.log(`  ${dir}: ${report.odometry.total} odom, ${report.planResults.total} plans, ` +
      `success ${(report.planResults.successRate * 100).toFixed(0)}%, ` +
      `max_v ${report.odometry.maxVelocity.toFixed(2)} m/s, ${health}`);
  }
  console.log(``);
}

if (import.meta.main) {
  cmdTestReport().then(() => process.exit(0));
}
