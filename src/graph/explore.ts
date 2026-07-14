import { StateGraph, START, END, Send, MemorySaver } from "@langchain/langgraph";
import { State, StateT } from "./state";
import { proposePlan } from "./nodes/propose-plan";
import { userCheckpoint } from "./nodes/checkpoint";
import { expandBatch } from "./nodes/expand-batch";
import { runContainer } from "./nodes/run-container";
import { analyzeResults } from "./nodes/analyze-results";
import { proposeNext } from "./nodes/propose-next";
import { generateReport } from "./nodes/generate-report";

function nextActionRouter(state: Record<string, any>): string | typeof END {
  const action = state.nextAction;
  if (action === "continue") return "executeBatch";
  if (action === "refine") return "refineBatch";
  if (action === "report" || action === "stop") return "generateReport";
  if (action === "approved") return "executeBatch";
  return END;
}

function checkpointRouter(state: Record<string, any>): string | typeof END {
  const action = state.nextAction;
  if (action === "approved") return "executeBatch";
  return END;
}

function fanOutRouter(state: Record<string, any>): (string | Send)[] | typeof END {
  const batch = state.currentBatchConfigs;
  if (!batch || batch.length === 0) return END;
  return batch.map((cfg: any) => new Send("runContainer", { config: cfg }));
}

export function buildExploreGraph() {
  const graph = new StateGraph(State)
    .addNode("proposePlan", proposePlan)
    .addNode("userCheckpoint", userCheckpoint)
    .addNode("executeBatch", expandBatch)
    .addNode("runContainer", runContainer)
    .addNode("analyzeResults", analyzeResults)
    .addNode("proposeNext", proposeNext)
    .addNode("actionCheckpoint", userCheckpoint)
    .addNode("refineBatch", expandBatch)
    .addNode("generateReport", generateReport)
    .addEdge(START, "proposePlan")
    .addEdge("proposePlan", "userCheckpoint")
    .addConditionalEdges("userCheckpoint", checkpointRouter)
    .addConditionalEdges("executeBatch", fanOutRouter)
    .addEdge("runContainer", "analyzeResults")
    .addEdge("analyzeResults", "proposeNext")
    .addEdge("proposeNext", "actionCheckpoint")
    .addConditionalEdges("actionCheckpoint", nextActionRouter)
    .addEdge("refineBatch", "executeBatch")
    .addEdge("generateReport", END);

  const checkpointer = new MemorySaver();
  return graph.compile({ checkpointer });
}
