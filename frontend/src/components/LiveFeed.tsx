import type { TestRunDisplay } from "../lib/types";

interface Props {
  tests: TestRunDisplay[];
}

export function LiveFeed({ tests }: Props) {
  const active = tests.filter((t) => t.active);

  return (
    <div style={{ padding: 12, borderTop: "1px solid #222" }}>
      <h3 style={{ fontSize: 13, color: "#888", marginBottom: 8, textTransform: "uppercase" }}>
        Live ({active.length})
      </h3>
      {active.length === 0 && (
        <p style={{ color: "#555", fontSize: 12 }}>No active containers</p>
      )}
      {active.map((t) => (
        <div key={t.id} style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4, fontSize: 12 }}>
          <span style={{
            width: 8, height: 8, borderRadius: "50%",
            background: t.color, display: "inline-block",
          }} />
          <span style={{ color: "#aaa" }}>{t.id}</span>
          <span style={{ color: "#555" }}>
            {t.positions.length > 0
              ? `[${t.positions[t.positions.length - 1].map((v) => v.toFixed(1)).join(", ")}]`
              : "waiting"}
          </span>
        </div>
      ))}
    </div>
  );
}
