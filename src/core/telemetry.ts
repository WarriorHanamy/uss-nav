import { CFG } from "./config";

interface JsonlEntry {
  timestamp?: number;
  [key: string]: unknown;
}

export async function readJsonl(testId: string, topic: string): Promise<JsonlEntry[]> {
  const filePath = `${CFG.testResultDir}/${testId}/${topic}.jsonl`;
  const file = Bun.file(filePath);
  const exists = await file.exists();
  if (!exists) return [];

  const text = await file.text();
  return text
    .trim()
    .split("\n")
    .filter(Boolean)
    .map((line) => JSON.parse(line));
}

export async function extractReplanCount(testId: string): Promise<number> {
  const entries = await readJsonl(testId, "plan_result");
  if (entries.length === 0) return 0;

  let count = 0;
  for (const e of entries) {
    if (e.planStatus === false || e.modifyStatus === true) count++;
  }
  return count;
}

export async function extractPlanSuccessRate(testId: string): Promise<number> {
  const entries = await readJsonl(testId, "plan_result");
  if (entries.length === 0) return 0;

  const successes = entries.filter((e) => e.planStatus === true).length;
  return successes / entries.length;
}

export async function extractExecutionDuration(testId: string): Promise<number> {
  const odom = await readJsonl(testId, "odom");
  if (odom.length < 2) return 0;

  const t0 = odom[0]?.timestamp ?? 0;
  const t1 = odom[odom.length - 1]?.timestamp ?? 0;
  return t1 - t0;
}

export async function extractAllMetrics(testId: string): Promise<Record<string, number>> {
  const [replanCount, successRate, execDuration] = await Promise.all([
    extractReplanCount(testId),
    extractPlanSuccessRate(testId),
    extractExecutionDuration(testId),
  ]);

  return {
    replan_count: replanCount,
    plan_success_rate: successRate,
    exec_duration_s: execDuration,
    odom_samples: (await readJsonl(testId, "odom")).length,
  };
}

export const AVAILABLE_METRICS = [
  {
    name: "replan_count",
    description: "Number of replanning triggers during flight",
    source: "plan_result.modifyStatus",
    unit: "count",
  },
  {
    name: "plan_success_rate",
    description: "Fraction of successful plan attempts",
    source: "plan_result.planStatus",
    unit: "ratio",
  },
  {
    name: "exec_duration_s",
    description: "Total flight execution time from first to last odom sample",
    source: "odom.timestamp",
    unit: "seconds",
  },
  {
    name: "odom_samples",
    description: "Total odometry samples collected",
    source: "odom",
    unit: "count",
  },
] as const;

export async function getAllMetricDefs() {
  return AVAILABLE_METRICS;
}
