import { cmdExplore } from "./explore";
import { cmdDashboard } from "./dashboard";

const USAGE = `
Usage: bun <command>

  explore      start interactive test exploration (LLM-guided)

  dashboard    start server + frontend (auto MQTT)
  server       start data server only
`.trim();

const CMD = process.argv[2];

async function main() {
  switch (CMD) {
    case "explore":
    case "test:explore":
      await cmdExplore();
      break;
    case "dashboard":
    case "dev":
      await cmdDashboard();
      break;
    case "server":
      await import("../server/index");
      break;
    case "help":
    case "--help":
    case "-h":
      console.log(USAGE);
      break;
    default:
      if (CMD) {
        console.error(`[ego-test] unknown: ${CMD}`);
      }
      console.log(USAGE);
      process.exit(CMD ? 1 : 0);
  }
}

main().catch((err) => {
  console.error("[ego-test] Error:", err);
  process.exit(1);
});
