import { ChatOpenAI } from "@langchain/openai";
import { LLM_CONFIG } from "../../core/config";

const llm = new ChatOpenAI({
  model: LLM_CONFIG.model,
  apiKey: LLM_CONFIG.apiKey,
  configuration: { baseURL: LLM_CONFIG.baseURL },
  temperature: LLM_CONFIG.temperature,
});

const PLAN_PROMPT = `You are a test planning assistant for the EGO Planner quadrotor trajectory planner.

Given the user's exploration intent, generate a concrete test plan.

Available parameters for testing:
- max_vel (m/s, range 0.2-2.0): maximum velocity
- max_acc (m/s^2, range 0.3-3.0): maximum acceleration
- flight_type (int 1-5): exploration strategy type
- obs_num (int 5-100): number of obstacles
- x_size (m, 10-100): map x dimension
- y_size (m, 10-100): map y dimension

Rules:
1. Start with a coarse grid of the primary parameters (use 2-3 values per param).
2. Group into batches of at most 4 configurations.
3. Each test runs for 120 seconds.
4. Use fixed defaults for non-explored params: obs_num=30, x_size=50, y_size=30, flight_type=2.
5. Generate batch IDs like: "b1-max_vel=0.5_max_acc=1.0"

Return ONLY valid JSON wrapped in \`\`\`json ... \`\`\`:
{
  "description": "Short plan summary",
  "batches": [[{ "id": "...", "params": {"max_vel": 0.5, "max_acc": 1.0, ...}, "duration": 120 }]],
  "estimatedMinutes": 10
}`;

export async function proposePlan(state: Record<string, any>): Promise<Record<string, any>> {
  const intent = state.intent;
  if (!intent) {
    return { plan: null, nextAction: "stop" };
  }

  const response = await llm.invoke([
    { role: "system", content: PLAN_PROMPT },
    { role: "user", content: `Exploration intent:\n${JSON.stringify(intent, null, 2)}` },
  ]);

  const content = typeof response.content === "string" ? response.content : JSON.stringify(response.content);
  const jsonMatch = content.match(/```json\n?([\s\S]*?)\n?```/);
  if (!jsonMatch) {
    return { plan: { description: content.slice(0, 200), batches: [], estimatedMinutes: 0 } };
  }

  try {
    const plan = JSON.parse(jsonMatch[1]);
    return { plan, currentBatchIndex: 0 };
  } catch {
    return { plan: { description: "Plan parsing failed", batches: [], estimatedMinutes: 0 } };
  }
}
