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
