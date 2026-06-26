export const enum TARGET_TYPE {
  MANUAL_TARGET = 1,
  EXPLORE_TARGET = 2,
  PRESET_TARGET = 3,
  REFERENCE_PATH = 4,
}

export interface TestScenario {
  id: string;
  params: Record<string, number[]>;
  fixed: Record<string, number>;
  duration: number;
}

export interface TestConfig {
  id: string;
  params: Record<string, number>;
  containerIndex: number;
  duration: number;
}

export interface TestRunStatus {
  scenarioId: string;
  configId: string;
  containerIndex: number;
  state: "starting" | "running" | "stopping" | "done" | "failed";
  startedAt: string;
  pid?: number;
}

export interface OdometrySample {
  ts: number;
  pos: [number, number, number];
  vel: [number, number, number];
  orient: [number, number, number, number];
}

export interface PlanResultSample {
  ts: number;
  goal: [number, number, number];
  plan_times: number;
  plan_status: boolean;
  modify_status: boolean;
}

export interface StateTriggerSample {
  ts: number;
  triggered: boolean;
}

export interface ExecFinishSample {
  ts: number;
  finished: boolean;
}

export interface DataDispSample {
  ts: number;
  a: number;
  b: number;
  c: number;
  d: number;
  e: number;
}

export interface TestData {
  testId: string;
  odometry: OdometrySample[];
  planResults: PlanResultSample[];
  stateTriggers: StateTriggerSample[];
  execFinishes: ExecFinishSample[];
  dataDisps: DataDispSample[];
  startTime: number;
  endTime: number;
}

export interface FrontendTestState {
  active: boolean;
  testId: string;
  odometry: OdometrySample[];
  planResults: PlanResultSample[];
  lastUpdate: number;
}

export type AssertStatus = "pass" | "fail" | "error";

export interface Assertion {
  name: string;
  status: AssertStatus;
  message: string;
  expected?: string;
  actual?: string;
}

export interface SmokeTestReport {
  testId: string;
  duration: number;
  startTime: number;
  endTime: number;
  assertions: Assertion[];
  passed: number;
  failed: number;
  errors: number;
  overall: AssertStatus;
}

export interface ScaleTestConfig {
  count: number;
  params: Record<string, number>;
  duration: number;
  batchSize: number;
  containerPrefix: string;
}

export interface ScaleTestReport {
  config: ScaleTestConfig;
  startTime: number;
  endTime: number;
  totalDuration: number;
  containersLaunched: number;
  containersSucceeded: number;
  containersFailed: number;
  containers: { id: string; status: string; sampleCount: number }[];
  resourceUsage: {
    avgCpu: number;
    peakCpu: number;
    avgMem: number;
    peakMem: number;
  };
}

export interface ValidationReport {
  testId: string;
  sampleCounts: Record<string, number>;
  odometry: {
    total: number;
    inBounds: boolean;
    maxVelocity: number;
    maxVelocityExceeded: boolean;
    minHeight: number;
  };
  planResults: {
    total: number;
    successCount: number;
    successRate: number;
    avgPlanTimes: number;
  };
  stateTriggers: {
    total: number;
    triggeredCount: number;
  };
  execFinishes: {
    total: number;
    finishedCount: number;
  };
}
