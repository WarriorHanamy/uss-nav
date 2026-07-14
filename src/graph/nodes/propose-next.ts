import { ChatOpenAI } from "@langchain/openai";
import { LLM_CONFIG } from "../../core/config";

const llm = new ChatOpenAI({
  model: LLM_CONFIG.model,
  apiKey: LLM_CONFIG.apiKey,
  configuration: { baseURL: LLM_CONFIG.baseURL },
  temperature: LLM_CONFIG.temperature,
});

const NEXT_PROMPT = `You are coordinating test exploration.

Based on the analysis and plan status, recommend the next action:
- "continue": there are still planned batches remaining
- "refine": dig deeper into a parameter region that showed high variance
- "report": enough data collected for a meaningful report
- "stop": cancel

For "refine", specify the exact parameter values to explore.

Return JSON wrapped in \`\`\`json ... \`\`\`:
{ "action": "continue"|"refine"|"report"|"stop", "refineParams": { "param": [values] } | null }`;

export async function proposeNext(state: Record<string, any>): Promise<Record<string, any>> {
  const batchIndex = state.currentBatchIndex ?? 0;
  const totalBatches = state.plan?.batches?.length ?? 0;
  const remaining = totalBatches - batchIndex;
  const analysis = state.analysis ?? "";

  const response = await llm.invoke([
    { role: "system", content: NEXT_PROMPT },
    {
      role: "user",
      content: `Analysis: ${analysis}\nRemaining batches: ${remaining}\nResults: ${(state.results ?? []).length} test(s)`,
    },
  ]);

  const content = typeof response.content === "string" ? response.content : JSON.stringify(response.content);
  const jsonMatch = content.match(/```json\n?([\s\S]*?)\n?```/);

  if (!jsonMatch) {
    return { nextAction: remaining > 0 ? "continue" : "report" };
  }

  try {
    const parsed = JSON.parse(jsonMatch[1]);
    return {
      nextAction: parsed.action,
      refineParams: parsed.refineParams ?? null,
    };
  } catch {
    return { nextAction: remaining > 0 ? "continue" : "report" };
  }
}
