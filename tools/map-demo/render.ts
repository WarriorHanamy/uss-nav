#!/usr/bin/env bun
import * as fs from "fs";
import * as path from "path";

// ============ Seeded RNG ============

class SeededRandom {
  private s: number;
  constructor(seed: number) { this.s = seed | 0; }
  next(): number {
    this.s = (this.s * 1664525 + 1013904223) | 0;
    return (this.s >>> 0) / 4294967296;
  }
}

// ============ 3D Perlin Noise ============

function hash3(ix: number, iy: number, iz: number): number {
  let h = (ix * 374761393 + iy * 668265263 + iz * 1274126177) | 0;
  h = ((h ^ (h >> 13)) * 1274126177) | 0;
  return ((h ^ (h >> 16)) >>> 0) / 4294967296;
}

function lerp(a: number, b: number, t: number) { return a + t * (b - a); }
function smst(t: number) { return t * t * (3 - 2 * t); }

function noise3(x: number, y: number, z: number): number {
  const ix = Math.floor(x), iy = Math.floor(y), iz = Math.floor(z);
  const fx = smst(x - ix), fy = smst(y - iy), fz = smst(z - iz);
  const v000 = hash3(ix, iy, iz), v100 = hash3(ix + 1, iy, iz);
  const v010 = hash3(ix, iy + 1, iz), v110 = hash3(ix + 1, iy + 1, iz);
  const v001 = hash3(ix, iy, iz + 1), v101 = hash3(ix + 1, iy, iz + 1);
  const v011 = hash3(ix, iy + 1, iz + 1), v111 = hash3(ix + 1, iy + 1, iz + 1);
  const x00 = lerp(v000, v100, fx), x10 = lerp(v010, v110, fx);
  const x01 = lerp(v001, v101, fx), x11 = lerp(v011, v111, fx);
  return lerp(lerp(x00, x10, fy), lerp(x01, x11, fy), fz);
}

function fbm3(x: number, y: number, z: number, octaves = 4): number {
  let val = 0, amp = 1, freq = 1, maxVal = 0;
  for (let i = 0; i < octaves; i++) {
    val += amp * noise3(x * freq, y * freq, z * freq); maxVal += amp;
    amp *= 0.5; freq *= 2;
  }
  return val / maxVal;
}

// ============ Types ============

const FREE = 0, OCCUPIED = 1, UNKNOWN = 2;

interface Vec3 { x: number; y: number; z: number; }

interface GridData {
  resolution: number; origin: Vec3; size: [number, number, number];
  occupancy: number[]; esdf: number[];
}
interface PolyData {
  id: number; center: Vec3; vertices: Vec3[];
  neighborIds: number[]; isFrontier: boolean;
}
interface CellData {
  index: [number, number, number]; center: Vec3;
  vmin: Vec3; vmax: Vec3; frontierCount: number; unknownCount: number;
}
interface ExportData { grid: GridData; polyhedra: PolyData[]; cells: CellData[]; }

// ============ GridMap ============

class GridMap {
  res: number; origin: Vec3; dims: [number, number, number];
  occ: Int32Array; esdf: Float64Array;

  constructor(res: number, origin: Vec3, size: Vec3) {
    this.res = res; this.origin = origin;
    const sx = Math.max(1, Math.round(size.x / res));
    const sy = Math.max(1, Math.round(size.y / res));
    const sz = Math.max(1, Math.round(size.z / res));
    this.dims = [sx, sy, sz];
    this.occ = new Int32Array(sx * sy * sz);
    this.occ.fill(UNKNOWN);
    this.esdf = new Float64Array(sx * sy * sz);
    this.esdf.fill(Infinity);
  }

  fi(ix: number, iy: number, iz: number) { return ix + iy * this.dims[0] + iz * this.dims[0] * this.dims[1]; }

  posToIdx(p: Vec3): [number, number, number] {
    return [Math.floor((p.x - this.origin.x) / this.res), Math.floor((p.y - this.origin.y) / this.res), Math.floor((p.z - this.origin.z) / this.res)];
  }
  idxToPos(idx: [number, number, number]): Vec3 {
    return { x: idx[0] * this.res + this.origin.x + this.res / 2, y: idx[1] * this.res + this.origin.y + this.res / 2, z: idx[2] * this.res + this.origin.z + this.res / 2 };
  }
  inBounds(idx: [number, number, number]) { return idx[0] >= 0 && idx[0] < this.dims[0] && idx[1] >= 0 && idx[1] < this.dims[1] && idx[2] >= 0 && idx[2] < this.dims[2]; }

  setOcc(p: Vec3, val: number) { const i = this.posToIdx(p); if (this.inBounds(i)) this.occ[this.fi(i[0], i[1], i[2])] = val; }

  addCircle(c: Vec3, r: number, val = OCCUPIED) {
    const r2 = r * r;
    for (let ix = 0; ix < this.dims[0]; ix++) for (let iy = 0; iy < this.dims[1]; iy++) {
      const p = this.idxToPos([ix, iy, 0]); const dx = p.x - c.x, dy = p.y - c.y;
      if (dx * dx + dy * dy < r2) this.occ[this.fi(ix, iy, 0)] = val;
    }
  }

  computeESDF() {
    this.esdf.fill(Infinity);
    const q: [number, number, number][] = [];
    for (let ix = 0; ix < this.dims[0]; ix++) for (let iy = 0; iy < this.dims[1]; iy++) for (let iz = 0; iz < this.dims[2]; iz++) {
      if (this.occ[this.fi(ix, iy, iz)] === OCCUPIED) { this.esdf[this.fi(ix, iy, iz)] = 0; q.push([ix, iy, iz]); }
    }
    const dirs: [number, number, number][] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    while (q.length) {
      const [ix, iy, iz] = q.shift()!;
      const d = this.esdf[this.fi(ix, iy, iz)] + this.res;
      for (const [dx, dy, dz] of dirs) {
        const nx = ix + dx, ny = iy + dy, nz = iz + dz;
        if (this.inBounds([nx, ny, nz]) && d < this.esdf[this.fi(nx, ny, nz)]) {
          this.esdf[this.fi(nx, ny, nz)] = d; q.push([nx, ny, nz]);
        }
      }
    }
  }

  toData(): GridData {
    return { resolution: this.res, origin: this.origin, size: this.dims, occupancy: Array.from(this.occ), esdf: Array.from(this.esdf) };
  }
}

// ============ MapManager ============

class MapManager {
  sml: GridMap; big: GridMap | null; cur: GridMap;
  constructor(sml: GridMap, big: GridMap | null = null) { this.sml = sml; this.big = big; this.cur = sml; }
  getOcc(p: Vec3): number { return this.cur.getOcc(p); }
}

// Patch for GridMap to support the getOcc method used by MapManager
GridMap.prototype.getOcc = function(p: Vec3): number {
  const i = this.posToIdx(p);
  return this.inBounds(i) ? this.occ[this.fi(i[0], i[1], i[2])] : UNKNOWN;
};

// ============ MapInterface (AStar) ============

class MapInterface {
  map: MapManager;
  constructor(m: MapManager) { this.map = m; }

  getOcc(p: Vec3) { return this.map.cur.getOcc(p); }
  isFree(p: Vec3) { const o = this.getOcc(p); return o === FREE || o === UNKNOWN; }

  sampleFreeSpace(count: number): Vec3[] {
    const gm = this.map.cur;
    const pts: Vec3[] = [];
    let tries = 0;
    while (pts.length < count && tries < count * 50) {
      const ix = Math.floor(Math.random() * gm.dims[0]);
      const iy = Math.floor(Math.random() * gm.dims[1]);
      const iz = Math.floor(Math.random() * gm.dims[2]);
      if (gm.occ[gm.fi(ix, iy, iz)] === FREE) pts.push(gm.idxToPos([ix, iy, iz]));
      tries++;
    }
    return pts;
  }

  searchPath(start: Vec3, goal: Vec3): Vec3[] | null {
    const gm = this.map.cur;
    const si = gm.posToIdx(start), gi = gm.posToIdx(goal);
    if (!gm.inBounds(si) || !gm.inBounds(gi)) return null;
    if (gm.occ[gm.fi(si[0], si[1], si[2])] === OCCUPIED || gm.occ[gm.fi(gi[0], gi[1], gi[2])] === OCCUPIED) return null;

    const dirs: [number, number, number][] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    const key = (ix: number, iy: number, iz: number) => `${ix},${iy},${iz}`;
    const open: [number, number, number, number, number, number][] = [];
    const cameFrom = new Map<string, string>();
    const gCost = new Map<string, number>();
    const sk = key(si[0], si[1], si[2]);
    gCost.set(sk, 0);
    open.push([0, si[0], si[1], si[2], gi[0], gi[1]]);

    while (open.length) {
      open.sort((a, b) => a[0] - b[0]);
      const [_, cx, cy, cz] = open.shift()!;
      const ck = key(cx, cy, cz);
      if (cx === gi[0] && cy === gi[1] && cz === gi[2]) {
        const path: Vec3[] = [];
        let curKey = ck;
        while (cameFrom.has(curKey)) {
          const [px, py, pz] = curKey.split(",").map(Number);
          path.push(gm.idxToPos([px, py, pz]));
          curKey = cameFrom.get(curKey)!;
        }
        path.push(gm.idxToPos(si));
        path.reverse();
        return path;
      }
      for (const [dx, dy, dz] of dirs) {
        const ni: [number, number, number] = [cx + dx, cy + dy, cz + dz];
        if (!gm.inBounds(ni)) continue;
        if (gm.occ[gm.fi(ni[0], ni[1], ni[2])] === OCCUPIED) continue;
        const nk = key(ni[0], ni[1], ni[2]);
        const ng = gCost.get(ck)! + 1;
        if (!gCost.has(nk) || ng < gCost.get(nk)!) {
          gCost.set(nk, ng);
          const h = Math.abs(ni[0] - gi[0]) + Math.abs(ni[1] - gi[1]) + Math.abs(ni[2] - gi[2]);
          cameFrom.set(nk, ck);
          open.push([ng + h, ni[0], ni[1], ni[2], gi[0], gi[1]]);
        }
      }
    }
    return null;
  }
}

// ============ Polyhedron / SkeletonGenerator ============

interface Poly {
  id: number; center: Vec3; vertices: Vec3[]; neighborIds: number[]; isFrontier: boolean;
}

class SkeletonGenerator {
  map: MapInterface; rng: SeededRandom;
  constructor(m: MapInterface, rng?: SeededRandom) { this.map = m; this.rng = rng || new SeededRandom(42); }

  generate(count: number): Poly[] {
    const centers = this.map.sampleFreeSpace(count);
    const polys: Poly[] = centers.map((c, i) => {
      const n = 6 + Math.floor(this.rng.next() * 7);
      const radius = 0.6 + this.rng.next() * 1.2;
      const verts: Vec3[] = [];
      for (let j = 0; j < n; j++) {
        const theta = Math.acos(2 * this.rng.next() - 1);
        const phi = 2 * Math.PI * this.rng.next();
        const r = radius * (0.6 + 0.4 * this.rng.next());
        verts.push({ x: c.x + r * Math.sin(theta) * Math.cos(phi), y: c.y + r * Math.sin(theta) * Math.sin(phi), z: c.z + r * Math.cos(theta) });
      }
      const isFrontier = this.checkFrontier(c);
      return { id: i, center: c, vertices: verts, neighborIds: [], isFrontier };
    });

    const neighborDist = 3.0;
    for (const p of polys) for (const q of polys) {
      if (p.id >= q.id) continue;
      const d = Math.hypot(p.center.x - q.center.x, p.center.y - q.center.y, p.center.z - q.center.z);
      if (d < neighborDist) { p.neighborIds.push(q.id); q.neighborIds.push(p.id); }
    }
    return polys;
  }

  private checkFrontier(c: Vec3): boolean {
    const gm = this.map.map.cur;
    const ci = gm.posToIdx(c);
    for (let dx = -4; dx <= 4; dx++) for (let dy = -4; dy <= 4; dy++) for (let dz = -4; dz <= 4; dz++) {
      const ni: [number, number, number] = [ci[0] + dx, ci[1] + dy, ci[2] + dz];
      if (gm.inBounds(ni) && gm.occ[gm.fi(ni[0], ni[1], ni[2])] === UNKNOWN) return true;
    }
    return false;
  }
}

// ============ UniformGrid ============

interface GridInfo {
  index: [number, number, number]; center: Vec3; vmin: Vec3; vmax: Vec3;
  frontierCount: number; unknownCount: number;
}

class UniformGrid {
  map: MapInterface; gridSize: number;
  constructor(m: MapInterface, gs = 2) { this.map = m; this.gridSize = gs; }

  computeCells(): GridInfo[] {
    const gm = this.map.map.cur;
    const gs = this.gridSize;
    const sx = Math.max(1, Math.round(gm.dims[0] * gm.res / gs));
    const sy = Math.max(1, Math.round(gm.dims[1] * gm.res / gs));
    const sz = Math.max(1, Math.round(gm.dims[2] * gm.res / gs));
    const cells: GridInfo[] = [];

    for (let ix = 0; ix < sx; ix++) for (let iy = 0; iy < sy; iy++) for (let iz = 0; iz < sz; iz++) {
      const vmin: Vec3 = { x: gm.origin.x + ix * gs, y: gm.origin.y + iy * gs, z: gm.origin.z + iz * gs };
      const vmax: Vec3 = { x: vmin.x + gs, y: vmin.y + gs, z: vmin.z + gs };
      const center: Vec3 = { x: vmin.x + gs / 2, y: vmin.y + gs / 2, z: vmin.z + gs / 2 };
      const info: GridInfo = { index: [ix, iy, iz], center, vmin, vmax, frontierCount: 0, unknownCount: 0 };
      this.populate(info, gm);
      cells.push(info);
    }
    return cells;
  }

  private populate(info: GridInfo, gm: GridMap) {
    const iMin = gm.posToIdx(info.vmin);
    const iMax = gm.posToIdx(info.vmax);
    for (let ix = iMin[0]; ix <= iMax[0]; ix++) for (let iy = iMin[1]; iy <= iMax[1]; iy++) for (let iz = iMin[2]; iz <= iMax[2]; iz++) {
      if (!gm.inBounds([ix, iy, iz])) continue;
      const o = gm.occ[gm.fi(ix, iy, iz)];
      if (o === UNKNOWN) { info.unknownCount++; if (this.hasFreeNeighbor(gm, [ix, iy, iz])) info.frontierCount++; }
    }
  }

  private hasFreeNeighbor(gm: GridMap, idx: [number, number, number]): boolean {
    const dirs: [number, number, number][] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
    for (const [dx, dy, dz] of dirs) {
      const ni: [number, number, number] = [idx[0] + dx, idx[1] + dy, idx[2] + dz];
      if (gm.inBounds(ni) && gm.occ[gm.fi(ni[0], ni[1], ni[2])] === FREE) return true;
    }
    return false;
  }
}

// ============ Build Demo ============

function buildDemo(): ExportData {
  const res = 0.25;
  const gm = new GridMap(res, { x: -5, y: -5, z: -0.125 }, { x: 10, y: 10, z: 3 });

  const noiseScale = 0.4;
  for (let ix = 0; ix < gm.dims[0]; ix++) for (let iy = 0; iy < gm.dims[1]; iy++) for (let iz = 0; iz < gm.dims[2]; iz++) {
    const p = gm.idxToPos([ix, iy, iz]);
    const val = fbm3(p.x * noiseScale, p.y * noiseScale, p.z * noiseScale, 4);
    if (val > 0.25) gm.occ[gm.fi(ix, iy, iz)] = OCCUPIED;
  }

  const kr = 4;
  const kc: Vec3 = { x: 0, y: 0, z: 1.5 };
  for (let ix = 0; ix < gm.dims[0]; ix++) for (let iy = 0; iy < gm.dims[1]; iy++) for (let iz = 0; iz < gm.dims[2]; iz++) {
    const p = gm.idxToPos([ix, iy, iz]);
    const dx = p.x - kc.x, dy = p.y - kc.y, dz = p.z - kc.z;
    if (dx * dx + dy * dy + dz * dz < kr * kr) {
      const fi = gm.fi(ix, iy, iz);
      if (gm.occ[fi] === UNKNOWN) gm.occ[fi] = FREE;
    }
  }

  gm.computeESDF();

  const mm = new MapManager(gm);
  const mi = new MapInterface(mm);
  const sg = new SkeletonGenerator(mi);
  const polys = sg.generate(45);
  const ug = new UniformGrid(mi, 1.5);
  const cells = ug.computeCells();

  return {
    grid: gm.toData(),
    polyhedra: polys.map((p) => ({
      id: p.id, center: p.center, vertices: p.vertices,
      neighborIds: p.neighborIds, isFrontier: p.isFrontier,
    })),
    cells: cells.map((c) => ({
      index: c.index, center: c.center, vmin: c.vmin, vmax: c.vmax,
      frontierCount: c.frontierCount, unknownCount: c.unknownCount,
    })),
  };
}

// ============ HTML Generation ============

function generateHTML(data: ExportData): string {
  const json = JSON.stringify(data);
  const occCount = data.grid.occupancy.filter((v) => v === OCCUPIED).length;
  const freeCount = data.grid.occupancy.filter((v) => v === FREE).length;
  const unkCount = data.grid.occupancy.filter((v) => v === UNKNOWN).length;
  const frontierPolys = data.polyhedra.filter((p) => p.isFrontier).length;
  const frontierCells = data.cells.filter((c) => c.frontierCount > 0).length;

  return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Map Demo — 5 Layers</title>
<script type="importmap">{
"imports":{
"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
}}</script>
<script>const DATA=${json};</script>
<style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{width:100%;height:100%;overflow:hidden;background:#1a1a2e;font-family:system-ui,sans-serif}
canvas{display:block}
#panel{position:fixed;top:16px;left:16px;z-index:10;background:rgba(0,0,0,0.78);border-radius:10px;padding:14px 18px;color:#ccc;font:13px/1.5 monospace;user-select:none;min-width:160px}
#panel h3{margin:0 0 8px 0;font-size:13px;color:#fff}
#panel label{display:flex;align-items:center;gap:8px;margin:4px 0;cursor:pointer}
#panel input[type=checkbox]{accent-color:#3498db}
#panel .badge{font-size:10px;color:#888;margin-left:auto}
#panel .count{font-size:10px;color:#666;display:block;padding-left:24px}
#legend{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);z-index:10;display:flex;gap:20px;padding:8px 18px;background:rgba(0,0,0,0.7);border-radius:8px;color:#aaa;font:12px/1.4 monospace;pointer-events:none}
#legend .item{display:flex;align-items:center;gap:5px}
#legend .swatch{width:12px;height:12px;border-radius:3px;border:1px solid rgba(255,255,255,0.1)}
#legend .sep{width:1px;height:18px;background:#444}
#info{position:fixed;top:16px;right:16px;z-index:10;padding:6px 14px;background:rgba(0,0,0,0.6);border-radius:6px;color:#666;font:11px monospace;pointer-events:none}
#table-btn{position:fixed;bottom:70px;left:50%;transform:translateX(-50%);z-index:10;padding:6px 16px;background:rgba(0,0,0,0.7);border:1px solid #444;border-radius:6px;color:#aaa;font:12px monospace;cursor:pointer}
#table-btn:hover{background:#333;color:#fff}
#table-overlay{position:fixed;inset:0;z-index:100;background:rgba(0,0,0,0.85);display:none;overflow:auto;padding:40px}
#table-overlay.open{display:block}
#table-overlay table{width:100%;border-collapse:collapse;font:12px/1.4 monospace;color:#ccc;margin-bottom:16px}
#table-overlay th,#table-overlay td{border:1px solid #333;padding:6px 10px;text-align:left}
#table-overlay th{background:#222;color:#fff;font-weight:600}
#table-overlay tr:hover td{background:rgba(255,255,255,0.03)}
#table-overlay .close{position:fixed;top:20px;right:30px;font:24px monospace;color:#888;cursor:pointer;background:none;border:none}
#table-overlay .close:hover{color:#fff}
</style>
</head>
<body>

<div id="panel">
<h3>Layers</h3>
<label><input type="checkbox" data-layer="occupied" checked><span style="color:#e74c3c">n GridMap Occupied</span><span class="badge">${occCount}</span></label>
<label><input type="checkbox" data-layer="free"><span style="color:#2ecc71">n GridMap Free</span><span class="badge">${freeCount}</span></label>
<label><input type="checkbox" data-layer="unknown" checked><span style="color:#777">n GridMap Unknown</span><span class="badge">${unkCount}</span></label>
<label><input type="checkbox" data-layer="esdf" checked><span style="color:#3498db">n ESDF</span></label>
<label><input type="checkbox" data-layer="polyhedra" checked><span style="color:#2ecc71">n Polyhedra</span><span class="badge">${data.polyhedra.length} (${frontierPolys} ftr)</span></label>
<label><input type="checkbox" data-layer="cells" checked><span style="color:#9b59b6">n UniformGrid Cells</span><span class="badge">${data.cells.length} (${frontierCells} ftr)</span></label>
</div>

<div id="info">Grid: ${data.grid.size[0]}x${data.grid.size[1]}x${data.grid.size[2]} @ ${data.grid.resolution}m</div>

<div id="legend">
<span class="item"><span class="swatch" style="background:#e74c3c"></span>Occupied</span>
<span class="item"><span class="swatch" style="background:#2ecc71;opacity:0.4"></span>GridMap Free</span>
<span class="item"><span class="swatch" style="background:#555"></span>Unknown</span>
<span class="item"><span class="swatch" style="background:linear-gradient(90deg,#e74c3c,#f1c40f,#3498db)"></span>ESDF</span>
<span class="item"><span class="swatch" style="background:#2ecc71"></span>Polyhedron</span>
<span class="item"><span class="swatch" style="background:#e67e22"></span>Frontier</span>
<span class="item"><span class="swatch" style="background:rgba(155,89,182,0.6)"></span>UniformGrid Cell</span>
</div>

<button id="table-btn">Comparison Table</button>
<div id="table-overlay">
<button class="close">&times;</button>
<h3 style="color:#fff;font:14px monospace;margin-bottom:12px">Map Representation Layers</h3>
<table>
<thead><tr><th>Layer</th><th>C++ Type</th><th>C++ File</th><th>Holder Data</th><th>Key API</th><th>Pipeline Role</th></tr></thead>
<tbody>
<tr><td><b style="color:#e74c3c">1</b></td><td>GridMap</td><td>plan_env/grid_map.h</td><td>occ_buf_ (体素占据) + dist_buf_ (ESDF) + 环形缓冲区</td><td>getOccupancy(), getDistance(), evaluateESDFWithGrad()</td><td>核心占据栅格 + ESDF；传感器融合</td></tr>
<tr><td><b style="color:#3498db">2</b></td><td>MapManager</td><td>plan_env/grid_map.h</td><td>sml_ (局部) + big_ (全局) 两个 GridMap + cur_</td><td>setMapUse(), getOcc() (委托到 cur_)</td><td>局部/全局双地图管理</td></tr>
<tr><td><b style="color:#f39c12">3</b></td><td>MapInterface</td><td>map_interface/map_interface.hpp</td><td>MapManager + AStar 路径搜索器</td><td>getOccupancy(), isInMap(), searchPath(), sampleFreeSpace()</td><td><b>共享 API Facade</b>；SceneGraph/EGO/FrontierFinder 用 shared_ptr 引用</td></tr>
<tr><td><b style="color:#2ecc71">4</b></td><td>Polyhedron / SkeletonGenerator</td><td>scene_graph/data_structure.h</td><td>顶点 vertices_ + 中心 + 邻居拓扑边 + 内含物体</td><td>expand() (射线投射生长), isFrontier()</td><td>自由空间拓扑骨架；SceneGraph 通过 MapInterface 查询栅格</td></tr>
<tr><td><b style="color:#9b59b6">5</b></td><td>UniformGrid / HGrid / GridInfo</td><td>active_perception/uniform_grid.h</td><td>粗网格单元 + frontier_num_ + unknown_num_</td><td>computeFrontiers(), getCellAt()</td><td>多分辨率前沿分组与视点代价</td></tr>
</tbody>
</table>
<h3 style="color:#fff;font:14px monospace;margin:20px 0 12px">Key Distinctions</h3>
<table>
<thead><tr><th>Layer</th><th>Ownership</th><th>Has Grid Data?</th><th>Shares Across Modules?</th><th>Consumers</th></tr></thead>
<tbody>
<tr><td>1 GridMap</td><td>MapManager</td><td>Yes (voxels + ESDF)</td><td>No (unique owner)</td><td>MapManager</td></tr>
<tr><td>2 MapManager</td><td>MapInterface</td><td>Indirect (via GridMap)</td><td>No (unique owner)</td><td>MapInterface</td></tr>
<tr><td>3 MapInterface</td><td>exploration_node</td><td>Indirect (via MapManager -> GridMap)</td><td><b>Yes — std::shared_ptr, zero-copy, same process</b></td><td>SceneGraph, FrontierFinder, EGOPlanner, ViewNode</td></tr>
<tr><td>4 Polyhedron</td><td>SkeletonGenerator</td><td>No (queries MapInterface only)</td><td>No (unique owner)</td><td>SceneGraph, LLM</td></tr>
<tr><td>5 UniformGrid</td><td>FrontierFinder / HGrid</td><td>No (queries MapInterface only)</td><td>No (unique owner)</td><td>FastExplorationFSM</td></tr>
</tbody>
</table>
</div>

<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const D = DATA;
const G = D.grid;
const res = G.resolution;
const origin = new THREE.Vector3(G.origin.x, G.origin.y, G.origin.z);

// --- Scene ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

const camera = new THREE.PerspectiveCamera(40, innerWidth/innerHeight, 0.1, 200);
camera.position.set(6, 8, 5);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
document.body.prepend(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0);
controls.update();

const ambient = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambient);
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(10, 20, 10);
scene.add(dir);

const gridHelper = new THREE.GridHelper(12, 8, 0x444466, 0x333355);
scene.add(gridHelper);
const axes = new THREE.AxesHelper(2);
scene.add(axes);

function flatToWorld(i) {
  const sx = G.size[0], sy = G.size[1];
  return new THREE.Vector3(
    origin.x + ((i % sx) + 0.5) * res,
    origin.y + (Math.floor(i / sx) % sy + 0.5) * res,
    origin.z + (Math.floor(i / (sx * sy)) + 0.5) * res
  );
}

// --- Layer 1: Occupied ---
const occGroup = new THREE.Group(); occGroup.name = "occupied";
const ocGeo = new THREE.BoxGeometry(res*0.9, res*0.9, res*0.9);
const ocMat = new THREE.MeshBasicMaterial({ color: 0xe74c3c });
const occList = G.occupancy.map((v,i)=>v===1?i:-1).filter(i=>i>=0);
const ocMesh = new THREE.InstancedMesh(ocGeo, ocMat, occList.length);
const dummy = new THREE.Object3D();
occList.forEach((i, idx) => {
  const p = flatToWorld(i); dummy.position.copy(p); dummy.updateMatrix(); ocMesh.setMatrixAt(idx, dummy.matrix);
});
ocMesh.instanceMatrix.needsUpdate = true;
occGroup.add(ocMesh);
scene.add(occGroup);

// --- Layer 2: Unknown ---
const unkGroup = new THREE.Group(); unkGroup.name = "unknown";
const unGeo = new THREE.BoxGeometry(res*0.85, res*0.85, res*0.85);
const unMat = new THREE.MeshBasicMaterial({ color: 0x555555, transparent: true, opacity: 0.3 });
const unkList = G.occupancy.map((v,i)=>v===2?i:-1).filter(i=>i>=0);
const unMesh = new THREE.InstancedMesh(unGeo, unMat, unkList.length);
unkList.forEach((i, idx) => {
  const p = flatToWorld(i); dummy.position.copy(p); dummy.updateMatrix(); unMesh.setMatrixAt(idx, dummy.matrix);
});
unMesh.instanceMatrix.needsUpdate = true;
unkGroup.add(unMesh);
scene.add(unkGroup);

// --- Layer Free ---
const freeGroup = new THREE.Group(); freeGroup.name = "free";
const frGeo = new THREE.BoxGeometry(res*0.85, res*0.85, res*0.85);
const frMat = new THREE.MeshBasicMaterial({ color: 0x2ecc71, transparent: true, opacity: 0.08 });
const freeList = G.occupancy.map((v,i)=>v===0?i:-1).filter(i=>i>=0);
const frMesh = new THREE.InstancedMesh(frGeo, frMat, freeList.length);
freeList.forEach((i, idx) => {
  const p = flatToWorld(i); dummy.position.copy(p); dummy.updateMatrix(); frMesh.setMatrixAt(idx, dummy.matrix);
});
frMesh.instanceMatrix.needsUpdate = true;
freeGroup.add(frMesh);
scene.add(freeGroup);

// --- Layer 3: ESDF Heatmap ---
const esdfGroup = new THREE.Group(); esdfGroup.name = "esdf";
const maxDist = G.esdf.filter(v=>isFinite(v)&&v<1e8).reduce((a,b)=>Math.max(a,b), 1);
const sliceZ = Math.floor(G.size[2] / 2);
const canvas = document.createElement("canvas");
canvas.width = G.size[0]; canvas.height = G.size[1];
const ctx = canvas.getContext("2d");
const img = ctx.createImageData(G.size[0], G.size[1]);
for (let iy=0; iy<G.size[1]; iy++) for (let ix=0; ix<G.size[0]; ix++) {
  const fi = ix + iy * G.size[0] + sliceZ * G.size[0] * G.size[1];
  const v = G.esdf[fi];
  const t = isFinite(v) && v<1e8 ? Math.min(v/(maxDist*0.5), 1) : 0;
  const idx = (iy * G.size[0] + ix) * 4;
  const h = (1 - t) * 240;
  const [R,Gg,B] = hslToRgb(h/360, 0.8, 0.5);
  img.data[idx]=R; img.data[idx+1]=Gg; img.data[idx+2]=B; img.data[idx+3]=180;
}
ctx.putImageData(img, 0, 0);
const tex = new THREE.CanvasTexture(canvas); tex.needsUpdate = true;
const plGeo = new THREE.PlaneGeometry(G.size[0]*res, G.size[1]*res);
const plMat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, side: THREE.DoubleSide });
const plane = new THREE.Mesh(plGeo, plMat);
plane.position.set(origin.x+G.size[0]*res/2, origin.y+G.size[1]*res/2, origin.z + (sliceZ + 0.5) * res);
esdfGroup.add(plane);
scene.add(esdfGroup);

function hslToRgb(h,s,l){
  let r,g,b; const c=(1-Math.abs(2*l-1))*s,x=c*(1-Math.abs((h*6)%2-1)),m=l-c/2;
  if(h<1/6){r=c;g=x;b=0}else if(h<2/6){r=x;g=c;b=0}else if(h<3/6){r=0;g=c;b=x}else if(h<4/6){r=0;g=x;b=c}else if(h<5/6){r=x;g=0;b=c}else{r=c;g=0;b=x}
  return [Math.round((r+m)*255),Math.round((g+m)*255),Math.round((b+m)*255)];
}

// --- Layer 4: Polyhedra ---
const polyGroup = new THREE.Group(); polyGroup.name = "polyhedra";
for (const p of D.polyhedra) {
  const pts = p.vertices.map(v=>new THREE.Vector3(v.x,v.y,v.z));
  if (pts.length<3) continue;
  const idxs = [];
  for (let i=0; i<pts.length; i++) for (let j=i+1; j<pts.length; j++) { idxs.push(i, j); }
  const g = new THREE.BufferGeometry().setFromPoints(pts); g.setIndex(idxs);
  const m = new THREE.LineBasicMaterial({ color: p.isFrontier?0xe67e22:0x2ecc71 });
  const seg = new THREE.LineSegments(g, m);
  polyGroup.add(seg);
  const dot = new THREE.Points(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(p.center.x,p.center.y,p.center.z)]),
    new THREE.PointsMaterial({ color: p.isFrontier?0xe67e22:0x2ecc71, size: 0.15, sizeAttenuation: true })
  );
  polyGroup.add(dot);
}
scene.add(polyGroup);

// --- Layer 5: UniformGrid cells ---
const cellGroup = new THREE.Group(); cellGroup.name = "cells";
const maxF = Math.max(...D.cells.map(c=>c.frontierCount), 1);
for (const c of D.cells) {
  if (c.frontierCount===0 && c.unknownCount===0) continue;
  const w = c.vmax.x-c.vmin.x, h = c.vmax.y-c.vmin.y, d = c.vmax.z-c.vmin.z;
  const t = c.frontierCount/maxF;
  const col = new THREE.Color().setHSL(0.7-t*0.5, 0.6, 0.3+t*0.2);
  const cg = new THREE.BoxGeometry(w*0.92, h*0.92, d*0.92);
  const cm = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.25+t*0.25, depthWrite: false });
  const cmesh = new THREE.Mesh(cg, cm);
  cmesh.position.set(c.center.x, c.center.y, c.center.z);
  cellGroup.add(cmesh);
}
scene.add(cellGroup);

// --- Toggles ---
document.querySelectorAll("[data-layer]").forEach(cb => {
  cb.addEventListener("change", () => {
    const obj = scene.getObjectByName(cb.dataset.layer);
    if (obj) obj.visible = cb.checked;
  });
});

// --- Resize ---
addEventListener("resize", () => {
  camera.aspect = innerWidth/innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// --- Table toggle ---
document.getElementById("table-btn").addEventListener("click", () => {
  document.getElementById("table-overlay").classList.add("open");
});
document.querySelector("#table-overlay .close").addEventListener("click", () => {
  document.getElementById("table-overlay").classList.remove("open");
});
document.getElementById("table-overlay").addEventListener("click", (e) => {
  if (e.target===e.currentTarget) e.target.classList.remove("open");
});

// --- Animate ---
function animate() { requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
animate();
</script>
</body>
</html>`;
}

// ============ Main ============

const outDir = path.resolve(__dirname, "../..", "_site");
if (!fs.existsSync(outDir)) fs.mkdirSync(outDir, { recursive: true });
const outPath = path.join(outDir, "map-demo.html");

console.log("Building demo data...");
const data = buildDemo();
console.log(`  Grid: ${data.grid.size[0]}x${data.grid.size[1]}x${data.grid.size[2]}`);
console.log(`  Occupied: ${data.grid.occupancy.filter(v=>v===OCCUPIED).length}`);
console.log(`  Free: ${data.grid.occupancy.filter(v=>v===FREE).length}`);
console.log(`  Unknown: ${data.grid.occupancy.filter(v=>v===UNKNOWN).length}`);
console.log(`  Polyhedra: ${data.polyhedra.length} (frontier: ${data.polyhedra.filter(p=>p.isFrontier).length})`);
console.log(`  Cells: ${data.cells.length} (frontier: ${data.cells.filter(c=>c.frontierCount>0).length})`);

console.log("Generating HTML...");
const html = generateHTML(data);
fs.writeFileSync(outPath, html, "utf-8");
console.log(`Written: ${outPath} (${(Buffer.byteLength(html) / 1024).toFixed(1)} KB)`);
