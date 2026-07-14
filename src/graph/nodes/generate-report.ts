import { ChatOpenAI } from "@langchain/openai";
import { LLM_CONFIG } from "../../core/config";

const llm = new ChatOpenAI({
  model: LLM_CONFIG.model,
  apiKey: LLM_CONFIG.apiKey,
  configuration: { baseURL: LLM_CONFIG.baseURL },
  temperature: LLM_CONFIG.temperature,
});

const REPORT_PROMPT = `Summarize exploration findings concisely.

1. What was tested and why
2. Key findings per parameter combination
3. Practical recommendations

Return JSON in \`\`\`json ... \`\`\`:
{ "summary": "...", "findings": ["..."], "suggestions": ["..."] }`;

export async function generateReport(state: Record<string, any>): Promise<Record<string, any>> {
  const intent = state.intent ?? {};
  const analysis = state.analysis ?? "";
  const results = state.results ?? [];

  const response = await llm.invoke([
    { role: "system", content: REPORT_PROMPT },
    {
      role: "user",
      content: [
        `Goal: ${intent.goal ?? "exploration"}`,
        `Analysis: ${analysis}`,
        `Results (${results.length} tests): ${JSON.stringify(results, null, 2)}`,
      ].join("\n"),
    },
  ]);

  const content = typeof response.content === "string" ? response.content : JSON.stringify(response.content);
  const jsonMatch = content.match(/```json\n?([\s\S]*?)\n?```/);

  if (jsonMatch) {
    try {
      const report = JSON.parse(jsonMatch[1]);
      console.log("\n=== Exploration Report ===");
      console.log(report.summary);
      for (const f of report.findings ?? []) console.log(`  - ${f}`);
      console.log("\nSuggestions:");
      for (const s of report.suggestions ?? []) console.log(`  * ${s}`);
      return { report };
    } catch { /* fall through */ }
  }

  console.log("\n=== Exploration Report ===");
  console.log(content);
  return { report: { summary: content, findings: [], suggestions: [] } };
}
