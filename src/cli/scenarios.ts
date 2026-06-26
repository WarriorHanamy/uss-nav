import type { TestScenario, TestConfig } from "../types/ego-test";

export const BUILTIN_SCENARIOS: TestScenario[] = [
  {
    id: "velocity_sweep",
    params: { max_vel: [0.3, 0.6, 1.0] },
    fixed: { flight_type: 2, max_acc: 1.0, obs_num: 30, x_size: 50, y_size: 30 },
    duration: 300,
  },
  {
    id: "map_variation",
    params: { obs_num: [10, 30, 60] },
    fixed: { flight_type: 2, max_vel: 0.6, max_acc: 1.0, x_size: 50, y_size: 30 },
    duration: 300,
  },
  {
    id: "aggressive_sweep",
    params: { max_vel: [0.5, 1.0, 1.5], max_acc: [0.5, 1.0, 2.0] },
    fixed: { flight_type: 2, obs_num: 30, x_size: 50, y_size: 30 },
    duration: 300,
  },
];

export function expandScenario(scenario: TestScenario): TestConfig[] {
  const configs: TestConfig[] = [];
  const paramKeys = Object.keys(scenario.params);

  function cartesian(acc: Record<string, number>, depth: number) {
    if (depth === paramKeys.length) {
      const params = { ...scenario.fixed, ...acc };
      const id = `${scenario.id}-${Object.values(acc).join("-").replace(/\./g, "_")}`;
      configs.push({ id, params, containerIndex: configs.length, duration: scenario.duration });
      return;
    }
    const key = paramKeys[depth];
    for (const val of scenario.params[key]) {
      cartesian({ ...acc, [key]: val }, depth + 1);
    }
  }

  cartesian({}, 0);
  return configs;
}
