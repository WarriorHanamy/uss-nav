export async function cmdTestStatus(): Promise<void> {
  const proc = Bun.spawn(["docker", "ps", "--filter", "name=ego-test-", "--format", "{{.Names}}\t{{.Status}}\t{{.CreatedAt}}"], {
    stdout: "pipe",
  });

  const output = await new Response(proc.stdout).text();
  const lines = output.trim().split("\n").filter(Boolean);

  if (lines.length === 0) {
    console.log("[test-status] No running test containers");
    return;
  }

  console.log("Running test containers:");
  console.log("────────────────────────────────────────────────");
  for (const line of lines) {
    const [name, status, created] = line.split("\t");
    console.log(`  ${name.padEnd(28)} ${status.padEnd(24)} ${created || ""}`);
  }
}
