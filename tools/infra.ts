const ROOT = new URL("..", import.meta.url).pathname;

const docker = {
  image: "ego-planner-sim",
  container: "ego-planner-sim",
} as const;

async function exec(command: string[], quiet = false): Promise<number> {
  const child = Bun.spawn(command, {
    cwd: ROOT,
    stdin: "inherit",
    stdout: quiet ? "ignore" : "inherit",
    stderr: quiet ? "ignore" : "inherit",
  });
  return await child.exited;
}

async function dockerBuild(): Promise<number> {
  return exec(["docker", "build", "-t", docker.image, "."]);
}

async function dockerRun(): Promise<number> {
  const display = Bun.env.DISPLAY;
  const home = Bun.env.HOME;

  if (!display) {
    console.error("DISPLAY is not set; RViz cannot connect to the host X server.");
    return 2;
  }
  if (!home) {
    console.error("HOME is not set; cannot mount the Xauthority file.");
    return 2;
  }

  // One project-owned RViz container: replace any previous instance.
  await exec(["docker", "rm", "-f", docker.container], true);

  return exec([
    "docker",
    "run",
    "--rm",
    "--name",
    docker.container,
    "--gpus",
    "all",
    "--ipc=host",
    "--security-opt",
    "seccomp=unconfined",
    "-e",
    `DISPLAY=${display}`,
    "-e",
    "__GLX_VENDOR_LIBRARY_NAME=nvidia",
    "-e",
    "QT_X11_NO_MITSHM=1",
    "-v",
    "/tmp/.X11-unix:/tmp/.X11-unix",
    "-v",
    `${home}/.Xauthority:/root/.Xauthority:ro`,
    docker.image,
  ]);
}

function usage(): void {
  console.error("Usage: bun infra docker <build|run>");
}

const [area, action, ...extra] = Bun.argv.slice(2);
if (area !== "docker" || extra.length > 0) {
  usage();
  process.exit(2);
}

const exitCode =
  action === "build"
    ? await dockerBuild()
    : action === "run"
      ? await dockerRun()
      : (usage(), 2);

process.exit(exitCode);
