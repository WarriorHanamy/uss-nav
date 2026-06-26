import { connectMqttClient } from "../core/mqtt";
import { CFG } from "../core/config";

const testData = new Map<string, Record<string, unknown[]>>();
const wsClients = new Set<Bun.ServerWebSocket<unknown>>();

function startServer() {
  Bun.serve({
    port: CFG.serverPort,
    fetch(req, server) {
      const url = new URL(req.url);

      if (url.pathname === "/api/tests") {
        const summary = Array.from(testData.entries()).map(([id, topics]) => ({
          id,
          topicCount: Object.keys(topics).length,
          sampleCount: Object.values(topics).reduce((s, v) => s + v.length, 0),
        }));
        return Response.json(summary);
      }

      if (url.pathname.startsWith("/api/test/")) {
        const testId = url.pathname.split("/").pop()!;
        const data = testData.get(testId) || {};
        return Response.json(data);
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
}

function broadcast(payload: object) {
  const msg = JSON.stringify(payload);
  for (const ws of wsClients) {
    try { ws.send(msg); } catch { /* ignore */ }
  }
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

    if (!testData.has(testId)) testData.set(testId, {});
    const bucket = testData.get(testId)!;
    if (!bucket[subtopic]) bucket[subtopic] = [];

    try {
      const parsed = JSON.parse(payload.toString());
      bucket[subtopic].push(parsed);
      if (bucket[subtopic].length > 10000) bucket[subtopic] = bucket[subtopic].slice(-5000);

      broadcast({ testId, subtopic, data: parsed });
    } catch {
      /* skip binary */
    }
  });

  client.on("error", (err) => console.warn(`[server] MQTT: ${err.message}`));
}

function main() {
  startMqttCollector();
  startServer();

  process.on("SIGINT", () => { console.log("\n[server] exit"); process.exit(0); });
}

if (import.meta.main) main();
