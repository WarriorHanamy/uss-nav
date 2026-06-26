import { useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera, Line as ThreeLine } from "@react-three/drei";
import * as THREE from "three";
import { DroneModel } from "./DroneModel";
import { LineChart, Line as RLine, CartesianGrid, XAxis, YAxis, Tooltip, ResponsiveContainer } from "recharts";
import type { TestDataBundle, OdometrySample, PosCmdSample } from "../lib/types";

interface Props {
  data: TestDataBundle | null;
  id: string;
}

/* ── Utility: flatten [x,y,z,...] array to Vector3 array ── */
function toVec3(pts: number[]): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  for (let i = 0; i + 2 < pts.length; i += 3)
    out.push(new THREE.Vector3(pts[i], pts[i + 1], pts[i + 2]));
  return out;
}

/* ── Layer Toggle ── */
type LayerKey = "occ" | "inf" | "body" | "traj" | "goal";

const LAYER_LABELS: Record<LayerKey, string> = {
  occ: "Occupancy",
  inf: "Inflated",
  body: "Body Cloud",
  traj: "Plan Trajectory",
  goal: "Goal",
};

/* ── 3D Scene with layers ── */
function Scene3D({ data }: { data: TestDataBundle }) {
  const odom = data.lastOdom ?? data.odom?.[data.odom!.length - 1];
  const pos: [number, number, number] = odom ? odom.pos : [0, 0, 0];
  const orient: [number, number, number, number] = odom ? odom.orient : [1, 0, 0, 0];
  const speed = odom ? Math.sqrt(odom.vel[0] ** 2 + odom.vel[1] ** 2 + odom.vel[2] ** 2) : 0;

  const trajPts = data.odom?.map((o) => [o.pos[0], o.pos[1], o.pos[2]] as [number, number, number]) ?? [];
  const lastGoal = data.lastPlanTraj ?? data.plan_traj?.[data.plan_traj!.length - 1];

  const [showLayers, setShowLayers] = useState<Record<LayerKey, boolean>>({
    occ: true, inf: true, body: true, traj: true, goal: true,
  });

  const Layers = useMemo(() => {
    const elems: React.ReactNode[] = [];

    // Trajectory line
    if (showLayers.traj && trajPts.length > 1) {
      elems.push(
        <ThreeLine key="traj" points={trajPts} color="#51cf66" lineWidth={2} transparent opacity={0.8} />,
      );
    }

    // Obstacles (occupancy)
    if (showLayers.occ && data.lastObstacles) {
      elems.push(
        <Points key="occ" pts={toVec3(data.lastObstacles.pts)} color="#e74c3c" size={0.15} />,
      );
    }

    // Inflated obstacles
    if (showLayers.inf && data.lastInflated) {
      elems.push(
        <Points key="inf" pts={toVec3(data.lastInflated.pts)} color="#f39c12" size={0.12} />,
      );
    }

    // Body pointcloud (sensor frame → world frame around drone)
    if (showLayers.body && data.lastBodyCloud) {
      elems.push(
        <Points key="body" pts={toVec3(data.lastBodyCloud.pts)} color="#aaa" size={0.06} />,
      );
    }

    // Goal point
    if (showLayers.goal && data.plan_result?.length) {
      const last = data.plan_result[data.plan_result.length - 1];
      elems.push(
        <mesh key="goal" position={[last.goal[0], last.goal[1], last.goal[2]]}>
          <sphereGeometry args={[0.25, 16, 16]} />
          <meshStandardMaterial color="#339af0" transparent opacity={0.7} />
        </mesh>,
      );
    }

    return elems;
  }, [data, showLayers, trajPts]);

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <Canvas style={{ width: "100%", height: "100%" }}>
        <PerspectiveCamera makeDefault position={[5, 4, 6]} />
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={0.8} />
        <gridHelper args={[40, 40, "#444", "#222"]} />
        <axesHelper args={[1.5]} />
        {Layers}
        <DroneModel position={pos} orientation={orient} speed={speed} />
        <OrbitControls enableDamping dampingFactor={0.1} maxDistance={50} minDistance={0.5} />
      </Canvas>
      {/* Layer toggles */}
      <div style={{ position: "absolute", bottom: 8, left: 8, display: "flex", gap: 6 }}>
        {(Object.keys(LAYER_LABELS) as LayerKey[]).map((k) => (
          <label key={k} style={{ fontSize: 11, color: "#aaa", cursor: "pointer", userSelect: "none" }}>
            <input type="checkbox" checked={showLayers[k]}
              onChange={() => setShowLayers({ ...showLayers, [k]: !showLayers[k] })} />
            {" "}{LAYER_LABELS[k]}
          </label>
        ))}
      </div>
    </div>
  );
}

/* ── Points helper component ── */
function Points({ pts, color, size }: { pts: THREE.Vector3[]; color: string; size: number }) {
  if (pts.length === 0) return null;
  const geo = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(
      pts.flatMap((p) => [p.x, p.y, p.z]), 3));
    return g;
  }, [pts]);
  return (
    <points geometry={geo}>
      <pointsMaterial color={color} size={size} sizeAttenuation />
    </points>
  );
}

/* ── Tracking Charts ── */
function TrackingCharts({ odom, posCmd }: { odom: OdometrySample[]; posCmd: PosCmdSample[] }) {
  const chartData = useMemo(() => {
    if (odom.length === 0) return [];
    const cmdMap = new Map(posCmd.map((p) => [Math.round(p.ts * 10), p]));
    return odom.filter((_, i) => i % 5 === 0).map((o) => {
      const cmd = cmdMap.get(Math.round(o.ts * 10));
      return {
        t: o.ts - odom[0].ts,
        px: o.pos[0], cmdPx: cmd?.pos[0] ?? null,
        py: o.pos[1], cmdPy: cmd?.pos[1] ?? null,
        pz: o.pos[2], cmdPz: cmd?.pos[2] ?? null,
        vx: o.vel[0], cmdVx: cmd?.vel[0] ?? null,
        vy: o.vel[1], cmdVy: cmd?.vel[1] ?? null,
        vz: o.vel[2], cmdVz: cmd?.vel[2] ?? null,
      };
    });
  }, [odom, posCmd]);

  if (chartData.length === 0)
    return <div style={{ color: "#555", padding: 16 }}>No tracking data</div>;

  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={chartData} margin={{ top: 6, right: 12, bottom: 2, left: 4 }}>
        <CartesianGrid stroke="#222" />
        <XAxis dataKey="t" tick={{ fontSize: 10, fill: "#888" }} label={{ value: "t [s]", position: "bottom", fill: "#888", fontSize: 10 }} />
        <YAxis tick={{ fontSize: 10, fill: "#888" }} />
        <Tooltip contentStyle={{ background: "#111", border: "1px solid #333", fontSize: 11 }} />
        <RLine dataKey="px" stroke="#ff6b6b" name="x" dot={false} isAnimationActive={false} strokeWidth={2} />
        <RLine dataKey="cmdPx" stroke="#ff6b6b" name="cmd x" dot={false} isAnimationActive={false} strokeDasharray="3 2" strokeWidth={1} />
        <RLine dataKey="py" stroke="#51cf66" name="y" dot={false} isAnimationActive={false} strokeWidth={2} />
        <RLine dataKey="cmdPy" stroke="#51cf66" name="cmd y" dot={false} isAnimationActive={false} strokeDasharray="3 2" strokeWidth={1} />
        <RLine dataKey="pz" stroke="#339af0" name="z" dot={false} isAnimationActive={false} strokeWidth={2} />
        <RLine dataKey="cmdPz" stroke="#339af0" name="cmd z" dot={false} isAnimationActive={false} strokeDasharray="3 2" strokeWidth={1} />
      </LineChart>
    </ResponsiveContainer>
  );
}

/* ── Main LiveView ── */
export function LiveView({ data, id }: Props) {
  const odom = data?.odom ?? [];
  const posCmd = data?.pos_cmd ?? [];
  const planResult = data?.plan_result ?? [];
  const successCount = planResult.filter((p) => p.plan_status).length;
  const successRate = planResult.length > 0 ? ((successCount / planResult.length) * 100).toFixed(0) : "N/A";
  const maxVel = odom.length > 0 ? Math.max(...odom.map((o) => Math.sqrt(o.vel[0] ** 2 + o.vel[1] ** 2 + o.vel[2] ** 2))) : 0;
  const lastGoal = planResult.length > 0 ? planResult[planResult.length - 1].goal : null;
  const lastOdom = odom.length > 0 ? odom[odom.length - 1] : null;

  return (
    <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>
      {/* 3D Scene */}
      <div style={{ flex: 1, display: "flex" }}>
        <div style={{ flex: 1, position: "relative" }}>
          {data ? <Scene3D data={data} /> : (
            <div style={{ display: "flex", alignItems: "center", justifyContent: "center", height: "100%", color: "#555" }}>
              Awaiting test data...
            </div>
          )}
        </div>

        {/* Side panel: Tracking Charts + Stats */}
        <div style={{ width: 360, borderLeft: "1px solid #333", display: "flex", flexDirection: "column", overflow: "hidden" }}>
          {/* Tracking */}
          <div style={{ flex: 1, minHeight: 180 }}>
            <h4 style={{ margin: "6px 10px", fontSize: 12, color: "#888", textTransform: "uppercase" }}>Tracking</h4>
            <div style={{ height: "calc(100% - 24px)" }}>
              <TrackingCharts odom={odom} posCmd={posCmd} />
            </div>
          </div>

          {/* Stats */}
          <div style={{ borderTop: "1px solid #222", padding: 10, fontSize: 12 }}>
            <h4 style={{ fontSize: 11, color: "#888", marginBottom: 6, textTransform: "uppercase" }}>Test: {id}</h4>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "2px 12px", color: "#aaa" }}>
              <span>Plans</span><span style={{ color: "#ccc" }}>{planResult.length}</span>
              <span>Success</span><span style={{ color: "#4caf50" }}>{successRate}%</span>
              <span>Max vel</span><span style={{ color: "#ccc" }}>{maxVel.toFixed(2)} m/s</span>
              <span>Goal</span><span style={{ color: "#ccc" }}>{lastGoal ? `${lastGoal[0].toFixed(1)}, ${lastGoal[1].toFixed(1)}` : "—"}</span>
              <span>Position</span><span style={{ color: "#ccc" }}>{lastOdom ? `${lastOdom.pos[0].toFixed(2)}, ${lastOdom.pos[1].toFixed(2)}` : "—"}</span>
            </div>
          </div>
        </div>
      </div>

      {/* Bottom row: body cloud, depth, stats */}
      <div style={{ height: 180, borderTop: "1px solid #333", display: "flex" }}>
        {/* Body cloud (top-down) */}
        <div style={{ flex: 1, borderRight: "1px solid #222", position: "relative", overflow: "hidden" }}>
          <h4 style={{ position: "absolute", top: 4, left: 8, fontSize: 11, color: "#888", zIndex: 2, textTransform: "uppercase" }}>Body Cloud</h4>
          <BodyCloudView pts={data?.lastBodyCloud?.pts ?? []} />
        </div>

        {/* Depth image */}
        <div style={{ flex: 1, borderRight: "1px solid #222", position: "relative" }}>
          <h4 style={{ position: "absolute", top: 4, left: 8, fontSize: 11, color: "#888", zIndex: 2, textTransform: "uppercase" }}>Body Depth</h4>
          {data?.lastBodyDepth ? (
            <img src={`data:image/jpeg;base64,${data.lastBodyDepth.jpeg_b64}`} alt="depth"
              style={{ width: "100%", height: "100%", objectFit: "contain" }} />
          ) : (
            <div style={{ display: "flex", alignItems: "center", justifyContent: "center", height: "100%", color: "#444", fontSize: 12 }}>
              Awaiting depth
            </div>
          )}
        </div>

        {/* Occupancy map (top-down 2D) */}
        <div style={{ flex: 1, position: "relative" }}>
          <h4 style={{ position: "absolute", top: 4, left: 8, fontSize: 11, color: "#888", zIndex: 2, textTransform: "uppercase" }}>Occ Map</h4>
          <OccMapView occ={data?.lastObstacles?.pts ?? []} infl={data?.lastInflated?.pts ?? []} />
        </div>
      </div>
    </div>
  );
}

/* ── BodyCloudView: 2D top-down projection ── */
function BodyCloudView({ pts }: { pts: number[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  useMemo(() => {
    const canvas = canvasRef.current;
    if (!canvas || pts.length < 3) return;
    const ctx = canvas.getContext("2d")!;
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0a0a0a";
    ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = "#aaa";
    for (let i = 0; i + 2 < pts.length && i < 3000; i += 3) {
      const x = (pts[i] * 10 + 0.5) * W;
      const y = (pts[i + 2] * 10 + 0.5) * H;
      if (x >= 0 && x < W && y >= 0 && y < H) {
        ctx.fillRect(x, y, 1, 1);
      }
    }
  }, [pts]);
  return <canvas ref={canvasRef} width={400} height={180} style={{ width: "100%", height: "100%" }} />;
}

/* ── OccMapView: top-down 2D occupancy ── */
function OccMapView({ occ, infl }: { occ: number[]; infl: number[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  useMemo(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d")!;
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0a0a0a";
    ctx.fillRect(0, 0, W, H);

    // Inflated (yellow, behind)
    if (infl.length > 0) {
      ctx.fillStyle = "#f39c1244";
      for (let i = 0; i + 2 < infl.length && i < 3000; i += 3) {
        const x = (infl[i] / 20 + 0.5) * W;
        const y = (infl[i + 1] / 20 + 0.5) * H;
        if (x >= 0 && x < W && y >= 0 && y < H) ctx.fillRect(x, y, 2, 2);
      }
    }

    // Occupancy (red, on top)
    if (occ.length > 0) {
      ctx.fillStyle = "#e74c3c88";
      for (let i = 0; i + 2 < occ.length && i < 3000; i += 3) {
        const x = (occ[i] / 20 + 0.5) * W;
        const y = (occ[i + 1] / 20 + 0.5) * H;
        if (x >= 0 && x < W && y >= 0 && y < H) ctx.fillRect(x, y, 2, 2);
      }
    }
  }, [occ, infl]);
  return <canvas ref={canvasRef} width={400} height={180} style={{ width: "100%", height: "100%" }} />;
}
