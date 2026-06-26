import { getRepoRoot } from "../core/workspace";
import { CFG } from "../core/config";

export async function cmdDockerBuild(noCache: boolean): Promise<void> {
  const root = getRepoRoot();

  const dockerfile = `${root}/docker/Dockerfile.test`;
  const tag = CFG.dockerImage;

  console.log(`[build] Building ${tag} from ${dockerfile} ...`);

  const args = ["build", "-f", dockerfile, "-t", tag, root];
  if (noCache) args.push("--no-cache");

  const proc = Bun.spawn(["docker", ...args], {
    cwd: root,
    stdin: "inherit",
    stdout: "inherit",
    stderr: "inherit",
  });

  const exitCode = await proc.exited;
  if (exitCode === 0) {
    console.log(`[build] ${tag} built successfully`);
  } else {
    console.error(`[build] docker build failed with exit code ${exitCode}`);
    process.exit(exitCode);
  }
}
