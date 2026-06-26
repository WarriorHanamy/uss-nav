import { useEffect, useRef, useState } from "react";
import { Scene3D } from "./components/Scene3D";
import { LiveFeed } from "./components/LiveFeed";
import { StatsGrid } from "./components/StatsGrid";
import type { OdometrySample, PlanResult, TestRunDisplay, WsMessage, TestRunMeta, TestDataBundle } from "./lib/types";

const COLORS = [
  "#ff6b6b", "#51cf66", "#339af0", "#fcc419",
  "#cc5de8", "#20c997", "#ff922b", "#f06595",
];

function buildDisplay(id: string, data?: TestDataBundle, color?: string): TestRunDisplay {
  const d: TestRunDisplay = {
    id,
    color: color || COLORS[Math.floor(Math.random() * COLORS.length)],
    positions: [],
    successRate: 0,
    planCount: 0,
    lastUpdate: 0,
    active: false,
  };
  if (data?.odom) {
    d.positions = data.odom.map((o) => o.pos);
    if (d.positions.length > 0) d.lastUpdate = Date.now();
  }
  if (data?.plan_result) {
    const pr = data.plan_result;
    d.planCount = pr.length;
    d.successRate = pr.filter((r) => r.plan_status).length / (pr.length || 1);
  }
  return d;
}

export default function App() {
  const [tests, setTests] = useState<Map<string, TestRunDisplay>>(new Map());
  const [connected, setConnected] = useState(false);
  const planResultsRef = useRef<Map<string, PlanResult[]>>(new Map());
  const colorMap = useRef<Map<string, string>>(new Map());

  function getColor(id: string): string {
    if (!colorMap.current.has(id)) {
      colorMap.current.set(id, COLORS[colorMap.current.size % COLORS.length]);
    }
    return colorMap.current.get(id)!;
  }

  // Load historical tests on mount
  useEffect(() => {
    fetch("/api/tests")
      .then((r) => r.json() as Promise<TestRunMeta[]>)
      .then((metaList) => {
        const loaded: TestRunDisplay[] = [];
        const promises = metaList.map((m) =>
          fetch(`/api/test/${m.id}`)
            .then((r) => r.json() as Promise<TestDataBundle>)
            .then((data) => {
              const d = buildDisplay(m.id, data, getColor(m.id));
              d.active = m.active;
              loaded.push(d);
            })
            .catch(() => {}),
        );
        return Promise.all(promises).then(() => loaded);
      })
      .then((historical) => {
        if (historical.length > 0) {
          setTests((prev) => {
            const next = new Map(prev);
            for (const t of historical) next.set(t.id, t);
            return next;
          });
        }
      })
      .catch(() => {});
  }, []);

  // WebSocket for live data
  useEffect(() => {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${protocol}//${location.hostname}:3000/ws`;
    const ws = new WebSocket(url);

    ws.onopen = () => setConnected(true);
    ws.onclose = () => setConnected(false);

    ws.onmessage = (ev) => {
      try {
        const msg: WsMessage = JSON.parse(ev.data);
        const { testId, subtopic, data } = msg;

        setTests((prev) => {
          const next = new Map(prev);
          const existing = next.get(testId) || {
            id: testId,
            color: getColor(testId),
            positions: [],
            successRate: 0,
            planCount: 0,
            lastUpdate: 0,
            active: true,
          };
          existing.active = true;

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

    return () => ws.close();
  }, []);

  // Mark stale tests as inactive
  useEffect(() => {
    const interval = setInterval(() => {
      setTests((prev) => {
        const next = new Map(prev);
        for (const [id, t] of next) {
          if (t.active && Date.now() - t.lastUpdate > 30000) {
            next.set(id, { ...t, active: false });
          }
        }
        return next;
      });
    }, 5000);
    return () => clearInterval(interval);
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
          {connected ? "Live" : "Offline"}
        </span>
        <span style={{ fontSize: 12, color: "#888" }}>
          {testArray.length} test(s)
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
