import { ChatOpenAI } from "@langchain/openai";
import { LLM_CONFIG } from "../../core/config";

const llm = new ChatOpenAI({
  model: LLM_CONFIG.model,
  apiKey: LLM_CONFIG.apiKey,
  configuration: { baseURL: LLM_CONFIG.baseURL },
  temperature: LLM_CONFIG.temperature,
});

const ANALYSIS_PROMPT = `You are analyzing EGO Planner test results.

Given the batch results and the exploration goal, identify:
1. Which parameter combinations show high variance or anomalies
2. Success/failure patterns
3. Whether any region needs deeper exploration

Keep analysis brief (2-3 sentences). Wrap JSON in \`\`\`json ... \`\`\`:
{ "analysis": "...", "suggestRefine": bool, "refineRegion": { "paramName": [values...] } | null }`;

export async function analyzeResults(state: Record<string, any>): Promise<Record<string, any>> {
  const results = state.results ?? [];
  const goal = state.intent?.goal ?? "";
  const batchIndex = (state.currentBatchIndex ?? 1) - 1;

  if (results.length === 0) {
    return { analysis: "No results to analyze.", batchResults: [] };
  }

  const summary = results
    .map((r: any) => `  ${r.testId}: exit=${r.exitCode}, ${JSON.stringify(r.metrics)}`)
    .join("\n");

  const response = await llm.invoke([
    { role: "system", content: ANALYSIS_PROMPT },
    { role: "user", content: `Goal: ${goal}\nBatch ${batchIndex} results:\n${summary}` },
  ]);

  const content = typeof response.content === "string" ? response.content : JSON.stringify(response.content);
  const jsonMatch = content.match(/```json\n?([\s\S]*?)\n?```/);
  const analysis = jsonMatch ? jsonMatch[1] : content;

  return {
    analysis,
    batchResults: [{
      batchIndex,
      configs: state.currentBatchConfigs ?? [],
      results: [...results],
    }],
  };
}
