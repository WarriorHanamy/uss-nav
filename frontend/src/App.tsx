import { useCallback, useEffect, useRef, useState } from "react";
import { LiveView } from "./components/LiveView";
import { PostProcess } from "./components/PostProcess";
import type {
  TestRunDisplay, WsMessage, TestRunMeta, TestDataBundle,
} from "./lib/types";

const COLORS = [
  "#ff6b6b", "#51cf66", "#339af0", "#fcc419",
  "#cc5de8", "#20c997", "#ff922b", "#f06595",
];

export default function App() {
  const [connected, setConnected] = useState(false);
  const [activeTab, setActiveTab] = useState<"live" | "post">("live");
  const [selectedId, setSelectedId] = useState<string>("");
  const [testIds, setTestIds] = useState<string[]>([]);
  const [liveBundle, setLiveBundle] = useState<TestDataBundle>({});
  const [historicalBundles, setHistoricalBundles] = useState<Map<string, TestDataBundle>>(new Map());
  const colorMap = useRef<Map<string, string>>(new Map());

  function getColor(id: string): string {
    if (!colorMap.current.has(id)) {
      colorMap.current.set(id, COLORS[colorMap.current.size % COLORS.length]);
    }
    return colorMap.current.get(id)!;
  }

  // Restore historical test list
  useEffect(() => {
    fetch("/api/tests")
      .then((r) => r.json() as Promise<TestRunMeta[]>)
      .then((metaList) => {
        const ids = metaList.map((m) => m.id);
        setTestIds((prev) => {
          const merged = new Set([...prev, ...ids]);
          return Array.from(merged);
        });
        if (!selectedId && ids.length > 0) setSelectedId(ids[0]);
      })
      .catch(() => {});
  }, []);

  // WebSocket: collect all subtopics per test ID
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

        setTestIds((prev) => {
          if (prev.includes(testId)) return prev;
          return [...prev, testId];
        });

        setLiveBundle((prev) => {
          const key = `last${subtopic.charAt(0).toUpperCase() + subtopic.slice(1)}` as keyof TestDataBundle;
          return {
            ...prev,
            [subtopic]: [...(prev[subtopic as keyof TestDataBundle] as unknown[] ?? []).slice(-10000), data],
            [key]: data,
            _ts: Date.now(),
          };
        });

        setSelectedId((prev) => prev || testId);
      } catch { /* ignore */ }
    };
    return () => ws.close();
  }, []);

  const activeBundle = liveBundle;
  const activeId = selectedId;

  // Secondary tab for post-process with full data
  const sortedTestIds = [...testIds].sort();

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100vh", background: "#111", color: "#ccc" }}>
      <header style={{
        padding: "6px 16px", background: "#1a1a2e", display: "flex",
        alignItems: "center", gap: 12, borderBottom: "1px solid #333",
      }}>
        <h1 style={{ fontSize: 15, color: "#fff", margin: 0 }}>EGO Planner</h1>
        <span style={{
          fontSize: 11, padding: "1px 6px", borderRadius: 3,
          background: connected ? "#2d4a2d" : "#4a2d2d",
          color: connected ? "#4caf50" : "#f44336",
        }}>
          {connected ? "Live" : "Offline"}
        </span>
        <div style={{ marginLeft: "auto", display: "flex", gap: 4, alignItems: "center" }}>
          <button onClick={() => setActiveTab("live")}
            style={tabBtnStyle(activeTab === "live")}>Live</button>
          <button onClick={() => setActiveTab("post")}
            style={tabBtnStyle(activeTab === "post")}>Post-Process</button>
        </div>
      </header>

      {activeTab === "live" ? (
        <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
          {/* Run selector during live (if multiple tests) */}
          {testIds.length > 1 && (
            <div style={{ padding: "4px 12px", display: "flex", gap: 8, alignItems: "center", borderBottom: "1px solid #222", fontSize: 12 }}>
              <span style={{ color: "#888" }}>Viewing:</span>
              <select value={activeId} onChange={(e) => setSelectedId(e.target.value)}
                style={{ background: "#111", color: "#ccc", border: "1px solid #333", borderRadius: 3, padding: "2px 6px", fontSize: 12 }}>
                {testIds.map((id) => <option key={id} value={id}>{id}</option>)}
              </select>
              <span style={{ color: "#555" }}>{testIds.length} test(s)</span>
            </div>
          )}
          <LiveView data={activeBundle} id={activeId} />
        </div>
      ) : (
        <PostProcess tests={sortedTestIds.map((id) => ({ id, color: getColor(id) }))} />
      )}
    </div>
  );
}

function tabBtnStyle(active: boolean): React.CSSProperties {
  return {
    padding: "3px 10px", fontSize: 12, borderRadius: 3, cursor: "pointer",
    background: active ? "#333" : "transparent",
    color: active ? "#fff" : "#888",
    border: "1px solid #444",
  };
}
