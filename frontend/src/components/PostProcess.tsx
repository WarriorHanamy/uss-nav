import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { TestDataBundle, TestRunMeta, OdometrySample, PosCmdSample, PlanResult } from "../lib/types";
import { LineChart, Line, CartesianGrid, XAxis, YAxis, Tooltip, ResponsiveContainer, Legend } from "recharts";

interface Props {
  tests: { id: string; color: string }[];
}

function flattenPts(pts: number[]): [number, number, number][] {
  const out: [number, number, number][] = [];
  for (let i = 0; i + 2 < pts.length; i += 3)
    out.push([pts[i], pts[i + 1], pts[i + 2]]);
  return out;
}

export function PostProcess({ tests }: Props) {
  const [selectedId, setSelectedId] = useState("");
  const [data, setData] = useState<TestDataBundle | null>(null);
  const [loading, setLoading] = useState(false);
  const [timeFrame, setTimeFrame] = useState(0);
  const [maxFrame, setMaxFrame] = useState(0);

  // Load data for selected run
  useEffect(() => {
    if (!selectedId) return;
    setLoading(true);
    fetch(`/api/test/${selectedId}`)
      .then((r) => r.json() as Promise<TestDataBundle>)
      .then((d) => {
        setData(d);
        const odom = d.odom ?? [];
        setMaxFrame(Math.max(odom.length - 1, 0));
        setTimeFrame(0);
      })
      .catch(() => setData(null))
      .finally(() => setLoading(false));
  }, [selectedId]);

  const odom = data?.odom ?? [];
  const planResult = data?.plan_result ?? [];
  const obstacles = data?.obstacles ?? [];
  const bodyCloud = data?.body_cloud ?? [];
  const bodyDepth = data?.body_depth ?? [];

  // Build time-series for charts
  const chartData = useMemo(() => {
    return odom.map((o, i) => {
      const pc = planResult.find((p) => Math.abs(p.ts - o.ts) < 0.1);
      return {
        t: o.ts,
        px: o.pos[0], py: o.pos[1], pz: o.pos[2],
        vx: o.vel[0], vy: o.vel[1], vz: o.vel[2],
        planStatus: pc ? 1 : 0,
      };
    });
  }, [odom, planResult]);

  const currentOdom = timeFrame < odom.length ? odom[timeFrame] : null;
  const currentObstaclePts = obstacles.length > 0
    ? flattenPts(obstacles[Math.min(timeFrame, obstacles.length - 1)].pts)
    : [];
  const currentBodyPts = bodyCloud.length > 0
    ? flattenPts(bodyCloud[Math.min(timeFrame, bodyCloud.length - 1)].pts)
    : [];

  return (
    <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
      {/* Run selector */}
      <div style={{ padding: "8px 16px", display: "flex", gap: 8, alignItems: "center", borderBottom: "1px solid #333" }}>
        <span style={{ color: "#888", fontSize: 13 }}>Run:</span>
        <select
          value={selectedId}
          onChange={(e) => setSelectedId(e.target.value)}
          style={{ background: "#111", color: "#ccc", border: "1px solid #333", padding: "4px 8px", borderRadius: 4 }}
        >
          <option value="">-- select --</option>
          {tests.map((t) => (
            <option key={t.id} value={t.id}>{t.id}</option>
          ))}
        </select>
        {loading && <span style={{ color: "#888", fontSize: 12 }}>loading...</span>}
        {data && <span style={{ color: "#888", fontSize: 12 }}>{odom.length} odom, {planResult.length} plans</span>}
      </div>

      {!data ? (
        <div style={{ flex: 1, display: "flex", alignItems: "center", justifyContent: "center", color: "#555" }}>
          Select a test run to inspect
        </div>
      ) : (
        <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "auto" }}>
          {/* 3D View */}
          <div style={{ height: 360, display: "flex", gap: 1 }}>
            <div style={{ flex: 1, background: "#0a0a0a", position: "relative", borderRadius: 6, overflow: "hidden" }}>
              <Trajectory3D odom={odom} obstacles={currentObstaclePts} bodyCloud={currentBodyPts} />
            </div>
            <div style={{ width: 320, background: "#0a0a0a", borderRadius: 6, overflow: "hidden" }}>
              {bodyDepth[Math.min(timeFrame, bodyDepth.length - 1)] ? (
                <DepthImage depth={bodyDepth[Math.min(timeFrame, bodyDepth.length - 1)]} />
              ) : (
                <div style={{ color: "#555", textAlign: "center", padding: 80 }}>No depth data</div>
              )}
            </div>
          </div>

          {/* Tracking Charts */}
          <div style={{ height: 200, background: "#0a0a0a", borderRadius: 6, marginTop: 4 }}>
            <TrackingCharts data={chartData} />
          </div>

          {/* Time slider */}
          <div style={{ padding: "4px 16px", display: "flex", gap: 8, alignItems: "center" }}>
            <span style={{ color: "#888", fontSize: 12 }}>Time:</span>
            <input
              type="range"
              min={0}
              max={maxFrame}
              value={timeFrame}
              onChange={(e) => setTimeFrame(Number(e.target.value))}
              style={{ flex: 1 }}
            />
            <span style={{ color: "#888", fontSize: 12 }}>
              {currentOdom ? `${currentOdom.pos[0].toFixed(1)}, ${currentOdom.pos[1].toFixed(1)}, ${currentOdom.pos[2].toFixed(1)}` : ""}
            </span>
          </div>
        </div>
      )}
    </div>
  );
}

/* ── Sub-components ────────────────────────────────────────── */

function Trajectory3D({ odom, obstacles, bodyCloud }: { odom: OdometrySample[]; obstacles: [number, number, number][]; bodyCloud: [number, number, number][] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || odom.length === 0) return;
    const ctx = canvas.getContext("2d")!;
    const W = canvas.width, H = canvas.height;

    // Compute bounds
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const o of odom) {
      if (o.pos[0] < minX) minX = o.pos[0];
      if (o.pos[0] > maxX) maxX = o.pos[0];
      if (o.pos[1] < minY) minY = o.pos[1];
      if (o.pos[1] > maxY) maxY = o.pos[1];
    }
    const pad = 2;
    minX -= pad; maxX += pad; minY -= pad; maxY += pad;
    const sx = W / (maxX - minX), sy = H / (maxY - minY);

    const tx = (x: number) => (x - minX) * sx;
    const ty = (y: number) => H - (y - minY) * sy;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#1a1a1a";
    ctx.fillRect(0, 0, W, H);

    // Draw obstacles
    if (obstacles.length > 0) {
      ctx.fillStyle = "#e74c3c44";
      for (const p of obstacles) {
        ctx.beginPath();
        ctx.arc(tx(p[0]), ty(p[1]), 2, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // Draw trajectory
    ctx.strokeStyle = "#51cf66";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < odom.length; i++) {
      const p = odom[i].pos;
      if (i === 0) ctx.moveTo(tx(p[0]), ty(p[1]));
      else ctx.lineTo(tx(p[0]), ty(p[1]));
    }
    ctx.stroke();

    // Draw start/end markers
    const first = odom[0].pos;
    const last = odom[odom.length - 1].pos;
    ctx.fillStyle = "#ff6b6b";
    ctx.beginPath();
    ctx.arc(tx(first[0]), ty(first[1]), 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#339af0";
    ctx.beginPath();
    ctx.arc(tx(last[0]), ty(last[1]), 5, 0, Math.PI * 2);
    ctx.fill();
  }, [odom, obstacles, bodyCloud]);

  return (
    <canvas ref={canvasRef} width={600} height={360}
      style={{ width: "100%", height: "100%" }} />
  );
}

function DepthImage({ depth }: { depth: { ts: number; jpeg_b64: string } }) {
  return (
    <div style={{ width: "100%", height: "100%", display: "flex", alignItems: "center", justifyContent: "center" }}>
      <img
        src={`data:image/jpeg;base64,${depth.jpeg_b64}`}
        alt="depth"
        style={{ maxWidth: "100%", maxHeight: "100%", objectFit: "contain" }}
      />
    </div>
  );
}

function TrackingCharts({ data }: { data: { t: number; px: number; py: number; pz: number; vx: number; vy: number; vz: number }[] }) {
  if (data.length === 0) return <div style={{ color: "#555", padding: 16 }}>No tracking data</div>;
  const sampled = data.filter((_, i) => i % 5 === 0);
  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={sampled} margin={{ top: 8, right: 16, bottom: 4, left: 8 }}>
        <CartesianGrid stroke="#222" />
        <XAxis dataKey="t" tick={{ fontSize: 10, fill: "#888" }} />
        <YAxis tick={{ fontSize: 10, fill: "#888" }} />
        <Tooltip contentStyle={{ background: "#111", border: "1px solid #333", fontSize: 12 }} />
        <Legend wrapperStyle={{ fontSize: 11, color: "#aaa" }} />
        <Line dataKey="px" stroke="#ff6b6b" name="pos X" dot={false} isAnimationActive={false} />
        <Line dataKey="py" stroke="#51cf66" name="pos Y" dot={false} isAnimationActive={false} />
        <Line dataKey="pz" stroke="#339af0" name="pos Z" dot={false} isAnimationActive={false} />
        <Line dataKey="vx" stroke="#fcc419" name="vel X" dot={false} isAnimationActive={false} />
        <Line dataKey="vy" stroke="#cc5de8" name="vel Y" dot={false} isAnimationActive={false} />
        <Line dataKey="vz" stroke="#20c997" name="vel Z" dot={false} isAnimationActive={false} />
      </LineChart>
    </ResponsiveContainer>
  );
}
