import type {
  Assertion,
  AssertStatus,
  OdometrySample,
  PlanResultSample,
  StateTriggerSample,
  ExecFinishSample,
  ValidationReport,
} from "../types/ego-test";

function ok(name: string, message: string): Assertion {
  return { name, status: "pass", message };
}

function fail(name: string, message: string, expected?: string, actual?: string): Assertion {
  return { name, status: "fail", message, expected, actual };
}

function error(name: string, message: string): Assertion {
  return { name, status: "error", message };
}

export function assertNonEmpty(
  arr: unknown[],
  label: string,
): Assertion {
  const n = arr.length;
  if (n === 0) return fail(label, `empty`, `> 0`, `0`);
  return ok(label, `${n} samples`);
}

export function assertOdometryBounds(
  samples: OdometrySample[],
  maxX: number,
  maxY: number,
  maxZ: number,
): Assertion[] {
  const results: Assertion[] = [];
  if (samples.length === 0) {
    results.push(fail("odom.bounds", "no samples to check"));
    return results;
  }
  const outOfBounds = samples.filter(
    (s) => Math.abs(s.pos[0]) > maxX || Math.abs(s.pos[1]) > maxY || s.pos[2] < -0.5 || s.pos[2] > maxZ,
  );
  if (outOfBounds.length > 0) {
    const worst = outOfBounds[0];
    results.push(fail("odom.bounds", `${outOfBounds.length} samples out of bounds`,
      `pos within [${maxX}, ${maxY}, ${maxZ}]`,
      `pos=(${worst.pos[0].toFixed(2)}, ${worst.pos[1].toFixed(2)}, ${worst.pos[2].toFixed(2)})`));
  } else {
    results.push(ok("odom.bounds", `all ${samples.length} samples within bounds`));
  }
  return results;
}

export function assertMaxVelocity(
  samples: OdometrySample[],
  maxVel: number,
  tolerance: number = 0.3,
): Assertion[] {
  const results: Assertion[] = [];
  if (samples.length === 0) {
    results.push(fail("odom.max_vel", "no samples to check"));
    return results;
  }
  const limit = maxVel + tolerance;
  const exceeded = samples.filter((s) => {
    const speed = Math.sqrt(s.vel[0] ** 2 + s.vel[1] ** 2 + s.vel[2] ** 2);
    return speed > limit;
  });
  if (exceeded.length > 0) {
    const worst = exceeded.reduce((a, b) => {
      const sa = Math.sqrt(a.vel[0] ** 2 + a.vel[1] ** 2 + a.vel[2] ** 2);
      const sb = Math.sqrt(b.vel[0] ** 2 + b.vel[1] ** 2 + b.vel[2] ** 2);
      return sa > sb ? a : b;
    });
    const worstSpeed = Math.sqrt(worst.vel[0] ** 2 + worst.vel[1] ** 2 + worst.vel[2] ** 2);
    results.push(fail("odom.max_vel", `${exceeded.length} samples exceed ${limit.toFixed(2)} m/s`,
      `speed ≤ ${limit.toFixed(2)}`,
      `max speed=${worstSpeed.toFixed(2)}`));
  } else {
    results.push(ok("odom.max_vel", `all speeds ≤ ${limit.toFixed(2)} m/s`));
  }
  return results;
}

export function assertPlanSuccessRate(
  results: PlanResultSample[],
  minRate: number = 0.5,
): Assertion[] {
  const a: Assertion[] = [];
  if (results.length === 0) {
    a.push(fail("plan.success_rate", "no plan results"));
    return a;
  }
  const successes = results.filter((r) => r.plan_status).length;
  const rate = successes / results.length;
  if (rate < minRate) {
    a.push(fail("plan.success_rate", `success rate ${(rate * 100).toFixed(1)}% < ${(minRate * 100).toFixed(0)}%`,
      `≥ ${(minRate * 100).toFixed(0)}%`,
      `${(rate * 100).toFixed(1)}% (${successes}/${results.length})`));
  } else {
    a.push(ok("plan.success_rate", `${(rate * 100).toFixed(1)}% (${successes}/${results.length})`));
  }
  return a;
}

export function assertStateTrigger(
  triggers: StateTriggerSample[],
): Assertion[] {
  const a: Assertion[] = [];
  if (triggers.length === 0) {
    a.push(fail("state_trigger", "no state triggers received — FSM may not be cycling"));
  } else {
    const triggered = triggers.filter((t) => t.triggered).length;
    a.push(ok("state_trigger", `${triggers.length} total, ${triggered} triggered`));
  }
  return a;
}

export function assertExecFinish(
  finishes: ExecFinishSample[],
): Assertion[] {
  const a: Assertion[] = [];
  if (finishes.length === 0) {
    a.push(fail("exec_finish", "no exec_finish events — trajectories may not complete"));
  } else {
    const done = finishes.filter((f) => f.finished).length;
    a.push(ok("exec_finish", `${finishes.length} events, ${done} completed`));
  }
  return a;
}

export function assertDataFlow(
  subtopics: Record<string, unknown[]>,
  required: string[],
): Assertion[] {
  const a: Assertion[] = [];
  for (const topic of required) {
    const arr = subtopics[topic];
    if (!arr || arr.length === 0) {
      a.push(fail(`data_flow.${topic}`, `no data for ${topic}`));
    } else {
      a.push(ok(`data_flow.${topic}`, `${arr.length} records`));
    }
  }
  return a;
}

export function buildValidationReport(
  testId: string,
  data: Record<string, unknown[]>,
  maxX: number,
  maxY: number,
  maxZ: number,
  maxVel: number,
): ValidationReport {
  const odom = (data["odom"] || []) as OdometrySample[];
  const planResults = (data["plan_result"] || []) as PlanResultSample[];
  const stateTriggers = (data["state_trigger"] || []) as StateTriggerSample[];
  const execFinishes = (data["exec_finish"] || []) as ExecFinishSample[];

  const maxSpeed = odom.length > 0
    ? Math.max(...odom.map((s) => Math.sqrt(s.vel[0] ** 2 + s.vel[1] ** 2 + s.vel[2] ** 2)))
    : 0;

  const outOfBounds = odom.filter(
    (s) => Math.abs(s.pos[0]) > maxX || Math.abs(s.pos[1]) > maxY || s.pos[2] < -0.5 || s.pos[2] > maxZ,
  ).length;

  const successes = planResults.filter((r) => r.plan_status).length;
  const avgPlanTimes = planResults.length > 0
    ? planResults.reduce((s, r) => s + r.plan_times, 0) / planResults.length
    : 0;

  return {
    testId,
    sampleCounts: Object.fromEntries(
      Object.entries(data).map(([k, v]) => [k, v.length]),
    ),
    odometry: {
      total: odom.length,
      inBounds: outOfBounds === 0,
      maxVelocity: maxSpeed,
      maxVelocityExceeded: maxSpeed > maxVel + 0.1,
      minHeight: odom.length > 0 ? Math.min(...odom.map((s) => s.pos[2])) : 0,
    },
    planResults: {
      total: planResults.length,
      successCount: successes,
      successRate: planResults.length > 0 ? successes / planResults.length : 0,
      avgPlanTimes,
    },
    stateTriggers: {
      total: stateTriggers.length,
      triggeredCount: stateTriggers.filter((t) => t.triggered).length,
    },
    execFinishes: {
      total: execFinishes.length,
      finishedCount: execFinishes.filter((f) => f.finished).length,
    },
  };
}
