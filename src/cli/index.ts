import { cmdDockerBuild } from "./docker-build";
import { cmdTestRun } from "./test-run";
import { cmdTestStatus } from "./test-status";
import { cmdTestStop } from "./test-stop";
import { cmdDashboard } from "./dashboard";

const USAGE = `
Usage: bun <command> [args]

  test:build [no-cache]    build ego-planner-test docker image
  test:run [scenario]      run test scenario (default: all)
  test:status              show running test containers
  test:stop [name]         stop test containers (default: all)

  dashboard                start server + frontend (auto MQTT)
  server                   start data server only
`.trim();

const CMD = process.argv[2];
const args = process.argv.slice(3);

async function main() {
  switch (CMD) {
    case "docker-build":
    case "test:build":
      await cmdDockerBuild(args.includes("no-cache"));
      break;
    case "test-run":
    case "test:run":
      await cmdTestRun(args[0]);
      break;
    case "test-status":
    case "test:status":
      await cmdTestStatus();
      break;
    case "test-stop":
    case "test:stop":
      await cmdTestStop(args[0]);
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
      if (CMD && !CMD.startsWith("docker") && !CMD.startsWith("view") && !CMD.startsWith("render") && CMD !== "infra") {
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
