import type { TestRunDisplay } from "../lib/types";

interface Props {
  tests: TestRunDisplay[];
}

export function StatsGrid({ tests }: Props) {
  const sorted = [...tests].sort((a, b) => b.lastUpdate - a.lastUpdate);

  return (
    <div style={{ padding: 12 }}>
      <h3 style={{ fontSize: 13, color: "#888", marginBottom: 8, textTransform: "uppercase" }}>
        Test Runs
      </h3>
      {sorted.length === 0 && (
        <p style={{ color: "#555", fontSize: 12 }}>No test data yet. Start a test run.</p>
      )}
      {sorted.map((t) => (
        <div
          key={t.id}
          style={{
            padding: "8px 10px",
            marginBottom: 6,
            borderRadius: 6,
            background: "#1a1a1a",
            borderLeft: `3px solid ${t.color}`,
            fontSize: 12,
          }}
        >
          <div style={{ color: "#ccc", fontWeight: "bold", marginBottom: 4 }}>
            {t.id}
          </div>
          <div style={{ color: "#888", display: "flex", justifyContent: "space-between" }}>
            <span>samples: {t.positions.length}</span>
            <span>plans: {t.planCount}</span>
          </div>
          <div style={{ color: "#888", display: "flex", justifyContent: "space-between" }}>
            <span>success: {t.successRate > 0 ? (t.successRate * 100).toFixed(1) : "N/A"}%</span>
            <span style={{ fontSize: 11, color: "#555" }}>
              {((Date.now() - t.lastUpdate) / 1000).toFixed(0)}s ago
            </span>
          </div>
        </div>
      ))}
    </div>
  );
}
