import { connectMqttClient } from "../core/mqtt";
import { getRepoRoot } from "../core/workspace";
import { CFG } from "../core/config";

function ensureMosquitto(): Promise<void> {
  return new Promise((resolve) => {
    const probe = connectMqttClient();
    const timeout = setTimeout(() => {
      probe.end(true);
      console.log("[dashboard] Mosquitto not reachable — starting...");
      const proc = Bun.spawnSync(["sudo", "systemctl", "start", "mosquitto"],
        { stdout: "inherit", stderr: "inherit" });
      if (proc.exitCode === 0) {
        console.log("[dashboard] Mosquitto started");
      } else {
        console.warn("[dashboard] Could not start Mosquitto; data collection disabled");
      }
      resolve();
    }, 2000);

    probe.on("connect", () => {
      clearTimeout(timeout);
      console.log("[dashboard] Mosquitto OK");
      probe.end(true);
      resolve();
    });

    probe.on("error", () => {});
  });
}

function waitForServer(base: string, retries = 15): Promise<void> {
  return new Promise((resolve) => {
    const check = () => {
      fetch(`${base}/api/tests`)
        .then(() => resolve())
        .catch(() => {
          if (--retries > 0) setTimeout(check, 500);
          else resolve(); // give up, let user rely on vite output
        });
    };
    check();
  });
}

export async function cmdDashboard(): Promise<void> {
  const root = getRepoRoot();
  const dashboardUrl = `http://localhost:5173`;
  const serverBase = `http://localhost:${CFG.serverPort}`;

  // 1. ensure MQTT broker
  await ensureMosquitto();

  // 2. start server (background)
  const server = Bun.spawn(["bun", "src/server/index.ts"], {
    cwd: root,
    stdout: "inherit",
    stderr: "inherit",
  });

  // 3. start Vite dev server (background)
  const vite = Bun.spawn(["bunx", "vite", "--host"], {
    cwd: root,
    stdout: "inherit",
    stderr: "inherit",
  });

  // 4. cleanup function
  let cleaningUp = false;
  const cleanup = () => {
    if (cleaningUp) return;
    cleaningUp = true;
    console.log("\n[dashboard] Shutting down...");
    server.kill();
    vite.kill();
    process.exit(0);
  };

  process.on("SIGINT", cleanup);
  process.on("SIGTERM", cleanup);

  // 5. wait for server readiness, then open browser
  await waitForServer(serverBase);
  console.log(`[dashboard] Opening ${dashboardUrl} ...`);
  Bun.spawn(["xdg-open", dashboardUrl], { stdout: "ignore", stderr: "ignore" });

  // 6. poll WS client count — when count stays 0 for 6s, shutdown
  let staleCycles = 0;

  const checkInterval = setInterval(async () => {
    try {
      const resp = await fetch(`${serverBase}/ws/clients`);
      const { count } = await resp.json() as { count: number };
      if (count === 0) {
        staleCycles++;
        if (staleCycles >= 3) {
          console.log("[dashboard] No browser connected. Stopping services.");
          clearInterval(checkInterval);
          cleanup();
        }
      } else {
        staleCycles = 0;
      }
    } catch {
      // server not ready yet
    }
  }, 2000);

  // avoid hanging if dashboard is the only process
  await Promise.race([
    server.exited.then(() => { console.error("[dashboard] Server exited unexpectedly"); cleanup(); }),
    vite.exited.then(() => { console.error("[dashboard] Vite exited unexpectedly"); cleanup(); }),
  ]);

  clearInterval(checkInterval);
  cleanup();
}
