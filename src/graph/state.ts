import { Annotation, START, END, Send, interrupt, Command } from "@langchain/langgraph";

const fallback = <T>(cur: T, update: T) => update ?? cur;

export const State = {
  userQuery: Annotation<string>(),
  messages: Annotation<{ role: string; content: string }[]>({
    reducer: (cur, update) => cur.concat(update),
    default: () => [],
  }),

  intent: Annotation<any>({ reducer: fallback, default: () => null }),
  plan: Annotation<any>({ reducer: fallback, default: () => null }),

  currentBatchIndex: Annotation<number>({
    reducer: (cur, update) => update ?? cur,
    default: () => 0,
  }),
  currentBatchConfigs: Annotation<any[]>({
    reducer: (cur, update) => update ?? cur,
    default: () => [],
  }),

  config: Annotation<any>({ reducer: fallback, default: () => null }),

  results: Annotation<any[]>({
    reducer: (cur, update) => cur.concat(update),
    default: () => [],
  }),

  batchResults: Annotation<any[]>({
    reducer: (cur, update) => cur.concat(update),
    default: () => [],
  }),

  analysis: Annotation<any>({ reducer: fallback, default: () => null }),
  nextAction: Annotation<any>({ reducer: fallback, default: () => null }),
  refineParams: Annotation<any>({ reducer: fallback, default: () => null }),

  report: Annotation<any>({ reducer: fallback, default: () => null }),
};

export type StateT = typeof State;
