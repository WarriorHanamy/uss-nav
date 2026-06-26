export async function cmdTestStop(nameArg?: string): Promise<void> {
  if (nameArg) {
    const containerName = nameArg.startsWith("ego-test-") ? nameArg : `ego-test-${nameArg}`;
    console.log(`[test-stop] Stopping ${containerName} ...`);
    await Bun.spawn(["docker", "rm", "-f", containerName], {
      stdout: "inherit",
      stderr: "inherit",
    }).exited;
    return;
  }

  const ps = Bun.spawn(["docker", "ps", "-q", "--filter", "name=ego-test-"], {
    stdout: "pipe",
  });
  const ids = (await new Response(ps.stdout).text()).trim().split("\n").filter(Boolean);

  if (ids.length === 0) {
    console.log("[test-stop] No test containers to stop");
    return;
  }

  console.log(`[test-stop] Stopping ${ids.length} container(s) ...`);
  await Bun.spawn(["docker", "rm", "-f", ...ids], {
    stdout: "inherit",
    stderr: "inherit",
  }).exited;
  console.log("[test-stop] All test containers stopped");
}
