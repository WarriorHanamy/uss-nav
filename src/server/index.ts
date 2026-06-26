import { mkdirSync, appendFileSync, readdirSync, readFileSync, existsSync, statSync } from "fs";
import { join } from "path";
import { connectMqttClient } from "../core/mqtt";
import { CFG } from "../core/config";

const RESULT_DIR = join(import.meta.dir, "../..", CFG.testResultDir);
const testData = new Map<string, Record<string, unknown[]>>();
const testLastActive = new Map<string, number>();
const wsClients = new Set<Bun.ServerWebSocket<unknown>>();
let nextColor = 0;
const COLORS = ["#ff6b6b", "#51cf66", "#339af0", "#fcc419", "#cc5de8", "#20c997", "#ff922b", "#f06595"];

function writeTestData(testId: string, subtopic: string, rawPayload: string) {
  const dir = join(RESULT_DIR, testId);
  mkdirSync(dir, { recursive: true });
  appendFileSync(join(dir, `${subtopic}.jsonl`), rawPayload + "\n");
}

function readTestData(testId: string): Record<string, unknown[]> {
  const result: Record<string, unknown[]> = {};
  const dir = join(RESULT_DIR, testId);
  if (!existsSync(dir)) return result;
  for (const file of readdirSync(dir)) {
    if (!file.endsWith(".jsonl")) continue;
    const subtopic = file.replace(".jsonl", "");
    result[subtopic] = [];
    for (const line of readFileSync(join(dir, file), "utf-8").trim().split("\n")) {
      if (!line) continue;
      try { result[subtopic].push(JSON.parse(line)); } catch {}
    }
  }
  return result;
}

function scanDiskTests(): { id: string; topicCount: number; sampleCount: number }[] {
  if (!existsSync(RESULT_DIR)) return [];
  return readdirSync(RESULT_DIR)
    .filter((name) => {
      try { return statSync(join(RESULT_DIR, name)).isDirectory(); } catch { return false; }
    })
    .map((id) => {
      const data = readTestData(id);
      const topicCount = Object.keys(data).length;
      const sampleCount = Object.values(data).reduce((s, v) => s + v.length, 0);
      return { id, topicCount, sampleCount };
    });
}

function startServer() {
  Bun.serve({
    port: CFG.serverPort,
    fetch(req, server) {
      const url = new URL(req.url);

      if (url.pathname === "/api/tests") {
        const disk = scanDiskTests();
        const mem = Array.from(testData.entries()).map(([id, topics]) => ({
          id,
          topicCount: Object.keys(topics).length,
          sampleCount: Object.values(topics).reduce((s, v) => s + v.length, 0),
        }));
        const merged = new Map<string, { id: string; topicCount: number; sampleCount: number; active: boolean }>();
        for (const t of disk) merged.set(t.id, { ...t, active: false });
        for (const t of mem) {
          const last = testLastActive.get(t.id) || 0;
          merged.set(t.id, { ...t, active: Date.now() - last < 30000 });
        }
        return Response.json(Array.from(merged.values()));
      }

      if (url.pathname.startsWith("/api/test/")) {
        const testId = url.pathname.split("/").pop()!;
        const disk = readTestData(testId);
        const mem = testData.get(testId) || {};
        const merged: Record<string, unknown[]> = {};
        for (const [k, v] of Object.entries(disk)) merged[k] = [...v];
        for (const [k, v] of Object.entries(mem)) merged[k] = [...(merged[k] || []), ...v];
        return Response.json(merged);
      }

      if (url.pathname === "/ws/clients") {
        return Response.json({ count: wsClients.size });
      }

      if (url.pathname === "/ws") {
        const upgraded = server.upgrade(req);
        if (!upgraded) return new Response("Upgrade failed", { status: 400 });
        return;
      }

      return new Response("Not found", { status: 404 });
    },
    websocket: {
      open(ws) {
        wsClients.add(ws);
        console.log(`[server] WS connected (${wsClients.size} total)`);
      },
      message() {},
      close(ws) {
        wsClients.delete(ws);
        console.log(`[server] WS disconnected (${wsClients.size} remaining)`);
      },
    },
  });

  console.log(`[server] Listening on :${CFG.serverPort}`);
  console.log(`[server] Results dir: ${RESULT_DIR}`);
}

function startMqttCollector() {
  const client = connectMqttClient();

  client.on("connect", () => {
    client.subscribe(`${CFG.topicPrefix}/+/+`, { qos: 0 });
    console.log(`[server] MQTT subscribed to ${CFG.topicPrefix}/+/+`);
  });

  client.on("message", (topic, payload) => {
    const parts = topic.split("/");
    if (parts.length < 3) return;
    const [, testId, subtopic] = parts;

    testLastActive.set(testId, Date.now());

    if (!testData.has(testId)) {
      testData.set(testId, {});
      nextColor = (nextColor + 1) % COLORS.length;
    }
    const bucket = testData.get(testId)!;
    if (!bucket[subtopic]) bucket[subtopic] = [];
    const raw = payload.toString();

    try {
      const parsed = JSON.parse(raw);
      bucket[subtopic].push(parsed);
      if (bucket[subtopic].length > 10000) bucket[subtopic] = bucket[subtopic].slice(-5000);
      writeTestData(testId, subtopic, raw);
      broadcast({ testId, subtopic, data: parsed });
    } catch {
      /* skip binary */
    }
  });

  client.on("error", (err) => console.warn(`[server] MQTT: ${err.message}`));
}

function broadcast(payload: object) {
  const msg = JSON.stringify(payload);
  for (const ws of wsClients) {
    try { ws.send(msg); } catch { /* ignore */ }
  }
}

function main() {
  startMqttCollector();
  startServer();

  process.on("SIGINT", () => { console.log("\n[server] exit"); process.exit(0); });
}

if (import.meta.main) main();
