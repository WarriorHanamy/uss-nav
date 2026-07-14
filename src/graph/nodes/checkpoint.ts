import { interrupt } from "@langchain/langgraph";

export async function userCheckpoint(state: Record<string, any>): Promise<Record<string, any>> {
  const plan = state.plan;
  const hasPlan = plan && plan.batches && plan.batches.length > 0;
  const analysis = state.analysis;
  const batchIndex = state.currentBatchIndex ?? 0;
  const totalBatches = plan?.batches?.length ?? 0;
  const hasMoreBatches = batchIndex < totalBatches;

  if (hasPlan && state.nextAction === null && !analysis) {
    const total = plan.batches.reduce((sum: number, b: any[]) => sum + b.length, 0);
    interrupt({
      type: "initial_plan",
      title: "Proposed Exploration Plan",
      description: plan.description,
      estimatedMinutes: plan.estimatedMinutes,
      totalTests: total,
      batches: plan.batches.length,
      prompt: `Type [approve] or [modify: ...] or [cancel]`,
    });
    return {};
  }

  if (analysis) {
    const remaining = hasMoreBatches ? totalBatches - batchIndex : 0;
    const recommended = state.nextAction ?? "report";

    interrupt({
      type: "action_decision",
      analysis,
      remainingBatches: remaining,
      recommended,
      prompt: `LLM recommends: [${recommended}]. Confirm or type: continue / refine / report / stop`,
    });
    return {};
  }

  return {};
}
