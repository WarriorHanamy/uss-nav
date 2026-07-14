import { CFG } from "../../core/config";

function expandRefineBatch(refineParams: Record<string, number[]>): any[] {
  const fixed: Record<string, number> = {
    flight_type: 2, obs_num: 30, x_size: 50, y_size: 30,
  };
  const keys = Object.keys(refineParams);
  if (keys.length === 0) return [];

  const configs: any[] = [];
  let idx = 0;

  function recurse(depth: number, acc: Record<string, number>) {
    if (depth === keys.length) {
      const label = Object.entries(acc).map(([k, v]) => `${k}=${v}`).join("_").replace(/\./g, "p");
      configs.push({
        id: "refine-" + label,
        params: { ...fixed, ...acc },
        containerIndex: idx++,
        duration: CFG.defaultDuration,
      });
      return;
    }
    const key = keys[depth];
    for (const val of refineParams[key]) {
      recurse(depth + 1, { ...acc, [key]: val });
    }
  }

  recurse(0, {});
  return configs.slice(0, CFG.maxContainers);
}

export async function expandBatch(state: Record<string, any>): Promise<Record<string, any>> {
  const refineParams = state.refineParams;
  const batches = state.plan?.batches;
  let batchIndex = state.currentBatchIndex ?? 0;

  if (refineParams && Object.keys(refineParams).length > 0) {
    const configs = expandRefineBatch(refineParams);
    return {
      currentBatchConfigs: configs,
      currentBatchIndex: batchIndex + 1,
      refineParams: null,
      nextAction: null,
    };
  }

  if (batches && batchIndex < batches.length) {
    const configs = batches[batchIndex];
    return {
      currentBatchConfigs: configs,
      currentBatchIndex: batchIndex + 1,
      nextAction: null,
    };
  }

  return { currentBatchConfigs: [], nextAction: "report" };
}
