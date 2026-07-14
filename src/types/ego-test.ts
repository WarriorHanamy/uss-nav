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
  timestamp: number;
  position: [number, number, number];
  velocity: [number, number, number];
  orientation: [number, number, number, number];
}

export interface PlanResult {
  timestamp: number;
  plannerGoal: [number, number, number];
  planTimes: number;
  planStatus: boolean;
  modifyStatus: boolean;
}

export interface StateTrigger {
  timestamp: number;
  state: number;
}

export interface TestData {
  testId: string;
  odometry: OdometrySample[];
  planResults: PlanResult[];
  states: StateTrigger[];
  startTime: number;
  endTime: number;
}

export interface FrontendTestState {
  active: boolean;
  testId: string;
  odometry: OdometrySample[];
  planResults: PlanResult[];
  lastUpdate: number;
}

export interface MetricDef {
  name: string;
  description: string;
  source: string;
  unit: string;
}

export interface ExplorationIntent {
  goal: string;
  focusParams: string[];
  paramRanges: Record<string, [number, number]>;
  targetMetrics: string[];
}

export interface ExplorationPlan {
  intent: ExplorationIntent;
  batches: TestConfig[][];
  description: string;
  estimatedMinutes: number;
}

export interface TestResult {
  testId: string;
  exitCode: number;
  metrics: Record<string, number>;
  error?: string;
}

export interface BatchResult {
  configs: TestConfig[];
  results: TestResult[];
}

export interface ExplorationReport {
  intent: ExplorationIntent;
  summary: string;
  batches: BatchResult[];
  findings: string[];
  suggestions: string[];
}

export type ExplorationAction =
  | { type: "continue" }
  | { type: "refine"; params: Record<string, number[]> }
  | { type: "report" }
  | { type: "stop"; reason: string };
