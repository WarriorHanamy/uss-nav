import { useCallback, useEffect, useRef, useState } from "react";
import { Scene3D } from "./components/Scene3D";
import { LiveFeed } from "./components/LiveFeed";
import { StatsGrid } from "./components/StatsGrid";
import type { OdometrySample, PlanResult, TestRunDisplay, WsMessage } from "./lib/types";

const COLORS = [
  "#ff6b6b", "#51cf66", "#339af0", "#fcc419",
  "#cc5de8", "#20c997", "#ff922b", "#f06595",
];

export default function App() {
  const [tests, setTests] = useState<Map<string, TestRunDisplay>>(new Map());
  const [connected, setConnected] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const planResultsRef = useRef<Map<string, PlanResult[]>>(new Map());
  const testsRef = useRef<Map<string, TestRunDisplay>>(tests);
  testsRef.current = tests;

  useEffect(() => {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${protocol}//${location.hostname}:3000/ws`;
    const ws = new WebSocket(url);

    ws.onopen = () => setConnected(true);
    ws.onclose = () => {
      setConnected(false);
      setTimeout(() => { /* reconnection is handled by server auto-reconnect */ }, 3000);
    };

    ws.onmessage = (ev) => {
      try {
        const msg: WsMessage = JSON.parse(ev.data);
        const { testId, subtopic, data } = msg;

        setTests((prev) => {
          const next = new Map(prev);
          const existing = next.get(testId) || {
            id: testId,
            color: COLORS[next.size % COLORS.length],
            positions: [],
            successRate: 0,
            planCount: 0,
            lastUpdate: 0,
          };

          if (subtopic === "odom") {
            const odom = data as unknown as OdometrySample;
            existing.positions = [...existing.positions.slice(-5000), odom.pos];
            existing.lastUpdate = Date.now();
          }

          if (subtopic === "plan_result") {
            const pr = data as unknown as PlanResult;
            const map = planResultsRef.current;
            const arr = map.get(testId) || [];
            arr.push(pr);
            if (arr.length > 1000) arr.shift();
            map.set(testId, arr);

            const total = arr.length;
            const success = arr.filter((r: PlanResult) => r.plan_status).length;
            existing.successRate = total > 0 ? success / total : 0;
            existing.planCount = total;
          }

          next.set(testId, existing);
          return next;
        });
      } catch { /* ignore */ }
    };

    wsRef.current = ws;
    return () => ws.close();
  }, []);

  const testArray = Array.from(tests.values());

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100vh" }}>
      <header style={{
        padding: "8px 16px", background: "#1a1a2e", display: "flex",
        alignItems: "center", gap: "16px", borderBottom: "1px solid #333",
      }}>
        <h1 style={{ fontSize: 16, color: "#fff" }}>EGO Planner Test</h1>
        <span style={{
          fontSize: 12, padding: "2px 8px", borderRadius: 4,
          background: connected ? "#2d4a2d" : "#4a2d2d",
          color: connected ? "#4caf50" : "#f44336",
        }}>
          {connected ? "Connected" : "Disconnected"}
        </span>
        <span style={{ fontSize: 12, color: "#888" }}>
          {testArray.length} test(s) · {testArray.reduce((s, t) => s + t.positions.length, 0)} samples
        </span>
      </header>

      <div style={{ flex: 1, display: "flex", overflow: "hidden" }}>
        <div style={{ flex: 1, position: "relative" }}>
          <Scene3D tests={testArray} />
        </div>
        <div style={{ width: 320, overflow: "auto", borderLeft: "1px solid #333" }}>
          <StatsGrid tests={testArray} />
          <LiveFeed tests={testArray} />
        </div>
      </div>
    </div>
  );
}
