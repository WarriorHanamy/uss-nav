import { ChatOpenAI } from "@langchain/openai";
import { Command } from "@langchain/langgraph";
import { LLM_CONFIG } from "../core/config";
import { buildExploreGraph } from "../graph/explore";

const llm = new ChatOpenAI({
  model: LLM_CONFIG.model,
  apiKey: LLM_CONFIG.apiKey,
  configuration: { baseURL: LLM_CONFIG.baseURL },
  temperature: 0.5,
});

const CONVERSATION_SYSTEM = `You are a test exploration assistant for the EGO Planner.

Your goal is to help the user define what they want to explore.

Available test parameters:
- max_vel (m/s, 0.2-2.0)
- max_acc (m/s^2, 0.3-3.0)
- flight_type (1-5)
- obs_num (5-100)
- x_size (m, 10-100)
- y_size (m, 10-100)

Available metrics: replan_count, plan_success_rate, exec_duration_s, odom_samples

Guide the conversation to clarify:
1. Which parameters they want to vary
2. What ranges
3. What metrics they care about
4. Their overall goal

When the intent is clear, output a JSON block with the exploration intent:
\`\`\`json
{
  "goal": "...",
  "focusParams": ["max_vel", "max_acc"],
  "paramRanges": {"max_vel": [0.3, 1.5], "max_acc": [0.5, 2.0]},
  "targetMetrics": ["replan_count", "plan_success_rate"]
}
\`\`\``;

function readLine(promptStr: string): Promise<string> {
  return new Promise((resolve) => {
    process.stdout.write(promptStr);
    process.stdin.once("data", (data) => resolve(String(data).trim()));
  });
}

async function conversationPhase(): Promise<Record<string, any>> {
  const messages: { role: string; content: string }[] = [
    { role: "system", content: CONVERSATION_SYSTEM },
    {
      role: "assistant",
      content:
        "你好！我来帮你探索 EGO Planner 的参数行为。\n\n你想研究哪些参数对什么结果的影响？例如：\n- max_vel 和 max_acc 对 replan 频率的影响\n- obs_num 对规划成功率的影响\n- 或者其他组合？",
    },
  ];

  console.log(`\n${messages[1].content}\n`);

  for (let turn = 0; turn < 20; turn++) {
    const userInput = await readLine("> ");
    if (!userInput || userInput === "exit" || userInput === "quit") {
      console.log("Exiting.");
      process.exit(0);
    }

    messages.push({ role: "user", content: String(userInput) });

    const response = await llm.invoke(messages);
    const content = typeof response.content === "string" ? response.content : JSON.stringify(response.content);
    messages.push({ role: "assistant", content });

    console.log(`\n${content}\n`);

    const jsonMatch = content.match(/```json\n?([\s\S]*?)\n?```/);
    if (jsonMatch) {
      try {
        const intent = JSON.parse(jsonMatch[1]);
        if (intent.focusParams && intent.focusParams.length > 0 && intent.paramRanges) {
          console.log("\nIntent captured. Starting exploration...");
          return { intent, messages };
        }
      } catch {
        // continue conversation
      }
    }
  }

  console.log("Too many turns, aborting.");
  process.exit(1);
}

async function executionPhase(intent: Record<string, any>) {
  const graph = buildExploreGraph();
  const threadId = `explore-${Date.now()}`;

  let state: Record<string, any> = {
    messages: [],
    intent,
    currentBatchIndex: 0,
    results: [],
    batchResults: [],
  };

  let currentStream = await graph.stream(state, { configurable: { thread_id: threadId } });
  let completed = false;

  while (!completed) {
    let interrupted = false;

    for await (const event of currentStream) {
      const interruptData = (event as any).__interrupt__;
      if (interruptData) {
        const info = Array.isArray(interruptData) ? interruptData[0] : interruptData;
        const prompt = info?.prompt ?? "Your choice:";

        if (info?.type === "initial_plan") {
          console.log(`\n=== ${info.title} ===`);
          console.log(`Plan: ${info.description}`);
          console.log(`Total: ${info.totalTests} tests across ${info.batches} batches`);
          console.log(`Estimated: ~${info.estimatedMinutes} min`);
        }
        if (info?.type === "action_decision") {
          console.log(`\nAnalysis: ${info.analysis}`);
          console.log(`LLM recommends: [${info.recommended}]`);
          if (info.remainingBatches > 0) {
            console.log(`Remaining batches: ${info.remainingBatches}`);
          }
        }

        const input = await readLine(`\n${prompt}\n> `);
        const action = (input || info?.recommended || "report").toLowerCase();
        state.nextAction = action;

        currentStream = await graph.stream(
          new Command({ resume: action, update: { nextAction: action } }),
          { configurable: { thread_id: threadId } }
        );
        interrupted = true;
        break;
      }

      for (const [, data] of Object.entries(event)) {
        if (data && typeof data === "object" && !Array.isArray(data)) {
          state = { ...state, ...data };
        }
      }
    }

    if (!interrupted) completed = true;
  }

  console.log("\nExploration complete.");
}

export async function cmdExplore(): Promise<void> {
  const { intent } = await conversationPhase();
  if (intent) {
    await executionPhase(intent);
  }
}
