import { cmdDockerBuild } from "./docker-build";
import { cmdTestRun } from "./test-run";
import { cmdTestStatus } from "./test-status";
import { cmdTestStop } from "./test-stop";
import { cmdDashboard } from "./dashboard";
import { cmdTestSmoke } from "./test-smoke";
import { cmdTestValidate } from "./test-validate";
import { cmdTestScale } from "./test-scale";
import { cmdTestReport } from "./test-report";

const USAGE = `
Usage: bun <command> [args]

  test:build [no-cache]      build ego-planner-test docker image
  test:run [scenario]        run test scenario (default: all)
  test:status                show running test containers
  test:stop [name]           stop test containers (default: all)

  test:smoke [id] [dur] [vel]
                             single end-to-end smoke test with assertions
  test:validate <test-id>    validate data from a completed test run
  test:scale [count] [dur] [batch]
                             concurrent scale stress test
  test:report                show summary of all recorded test runs

  dashboard                  start server + frontend (auto MQTT)
  server                     start data server only
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
    case "test-smoke":
    case "test:smoke": {
      const smokeReport = await cmdTestSmoke(args[0], args[1] ? parseInt(args[1]) : 60, args[2] ? parseFloat(args[2]) : 0.6);
      if (smokeReport.overall !== "pass") process.exit(1);
      break;
    }
    case "test-validate":
    case "test:validate":
      if (!args[0]) { console.error("[test:validate] <test-id> required"); process.exit(1); }
      await cmdTestValidate(args[0]);
      break;
    case "test-scale":
    case "test:scale":
      await cmdTestScale(
        args[0] ? parseInt(args[0]) : 4,
        args[1] ? parseInt(args[1]) : 120,
      );
      break;
    case "test-report":
    case "test:report":
      await cmdTestReport();
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
