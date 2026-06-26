import { readFileSync, existsSync } from "fs";
import { join } from "path";
import type { TestScenario, TestConfig } from "../types/ego-test";
import { parseYaml } from "./yaml";

export const BUILTIN_SCENARIOS: TestScenario[] = [
  {
    id: "smoke",
    params: { max_vel: [0.6] },
    fixed: { flight_type: 2, max_acc: 1.0, obs_num: 30, x_size: 50, y_size: 30 },
    duration: 60,
  },
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
  {
    id: "scale_4",
    params: { max_vel: [0.6] },
    fixed: { flight_type: 2, max_acc: 1.0, obs_num: 30, x_size: 50, y_size: 30 },
    duration: 120,
  },
];

export function loadScenarios(yamlPath?: string): TestScenario[] {
  if (!yamlPath) return BUILTIN_SCENARIOS;

  const path = join(process.cwd(), yamlPath);
  if (!existsSync(path)) {
    console.warn(`[scenarios] YAML not found: ${path}, using built-in`);
    return BUILTIN_SCENARIOS;
  }

  const content = readFileSync(path, "utf-8");
  const doc = parseYaml(content);

  if (!doc.scenarios || !Array.isArray(doc.scenarios)) {
    console.warn(`[scenarios] no 'scenarios' list in YAML, using built-in`);
    return BUILTIN_SCENARIOS;
  }

  return (doc.scenarios as Record<string, unknown>[]).map((raw: Record<string, unknown>) => ({
    id: raw.id as string,
    params: raw.params as Record<string, number[]>,
    fixed: raw.fixed as Record<string, number>,
    duration: raw.duration as number,
  }));
}

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
