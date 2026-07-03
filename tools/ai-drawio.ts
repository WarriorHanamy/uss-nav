import { copyFileSync, existsSync, readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

const upstream = "https://github.com/DayuanJiang/next-ai-draw-io.git";
const pinnedRef = "5bfd7b24680cbf807b5ea17d83f05edc33c6f5f2";
const projectRoot = resolve(import.meta.dir, "..");
const appDir = resolve(projectRoot, "tools/.cache/next-ai-draw-io");
const command = process.argv[2] ?? "dev";
const port = process.env.NEXT_AI_DRAWIO_PORT ?? "6002";

function applyUssNavOverlay() {
  copyFileSync(
    resolve(projectRoot, "tools/ai-drawio/uss-nav.drawio"),
    resolve(appDir, "public/uss-nav.drawio"),
  );

  const pagePath = resolve(appDir, "app/[lang]/page.tsx");
  let page = readFileSync(pagePath, "utf8");
  // Migrate the former imperative post-load injection. It raced with the
  // iframe's own initial empty load and could leave users on blank Page-1.
  page = page.replace(
    /\n    \/\/ USS_NAV_DEFAULT_DIAGRAM:[\s\S]*?\n    const handleDarkModeChange = \(\) => \{/,
    "\n    const handleDarkModeChange = () => {",
  );
  if (!page.includes("USS_NAV_XML_PROP")) {
    page = page.replace(
      "    const chatPanelRef = useRef<ImperativePanelHandle>(null)",
      `    // USS_NAV_XML_PROP: feed the complete multi-page file through react-drawio's own load lifecycle.\n    const [ussNavXml, setUssNavXml] = useState<string>(\"\")\n\n    useEffect(() => {\n        void fetch(\"/uss-nav.drawio\")\n            .then((response) => {\n                if (!response.ok) throw new Error(\`HTTP \${response.status}\`)\n                return response.text()\n            })\n            .then(setUssNavXml)\n            .catch((error) => console.error(\"Failed to load USS-NAV architecture\", error))\n    }, [])\n\n    const chatPanelRef = useRef<ImperativePanelHandle>(null)`,
    );
    page = page.replace(
      "                                        autosave\n",
      "                                        xml={ussNavXml || undefined}\n                                        autosave\n",
    );
  }
  if (!page.includes("USS_NAV_REAPPLY_AFTER_SESSION")) {
    page = page.replace(
      "    const handleDarkModeChange = () => {",
      `    // USS_NAV_REAPPLY_AFTER_SESSION: ChatPanel may restore an empty IndexedDB\n    // session after draw.io's initial load. Reapply only after the editor reports ready.\n    useEffect(() => {\n        if (!isDrawioReady || !ussNavXml) return\n        const timer = window.setTimeout(() => {\n            drawioRef.current?.load({ xml: ussNavXml, autosave: true })\n        }, 750)\n        return () => window.clearTimeout(timer)\n    }, [isDrawioReady, ussNavXml, drawioRef])\n\n    const handleDarkModeChange = () => {`,
    );
  }
  page = page.replace(
    "    const [isChatVisible, setIsChatVisible] = useState(true)",
    "    const [isChatVisible, setIsChatVisible] = useState(false)",
  );
  page = page.replace(
    "                    collapsedSize={isMobile ? 0 : 3}",
    "                    collapsedSize={0}",
  );
  page = page.replace(
    /\n    \/\/ USS_NAV_NO_SIDEBAR:[\s\S]*?\n    \}, \[\]\)/,
    "",
  );
  page = page.replace(
    "                    defaultSize={isMobile ? 50 : 0}",
    "                    defaultSize={isMobile ? 50 : 33}",
  );
  page = page.replace(
    "        if (panel) {",
    "        if (!panel) {\n            setIsChatVisible(true)\n            return\n        }\n        if (panel) {",
  );
  if (!page.includes("USS_NAV_NO_SIDEBAR_RENDER")) {
    page = page.replace(
      "                <ResizableHandle withHandle />\n\n                {/* Chat Panel */}",
      "                {/* USS_NAV_NO_SIDEBAR_RENDER: mount chat only on explicit toggle. */}\n                {isChatVisible && (<>\n                <ResizableHandle withHandle />\n\n                {/* Chat Panel */}",
    );
    page = page.replace(
      "                </ResizablePanel>\n            </ResizablePanelGroup>",
      "                </ResizablePanel>\n                </>)}\n            </ResizablePanelGroup>",
    );
  }
  writeFileSync(pagePath, page);

  const chatPath = resolve(appDir, "components/chat-panel.tsx");
  let chat = readFileSync(chatPath, "utf8");
  chat = chat.replace(
    'const DEBUG = process.env.NODE_ENV === "development"',
    'const DEBUG = process.env.NEXT_PUBLIC_SHOW_DEV_TOOLS === "1"',
  );
  writeFileSync(chatPath, chat);
}

function run(args: string[], cwd = projectRoot) {
  const result = Bun.spawnSync(args, {
    cwd,
    env: {
      ...process.env,
      BUN_TMPDIR: process.env.BUN_TMPDIR ?? "/tmp",
    },
    stdin: "inherit",
    stdout: "inherit",
    stderr: "inherit",
  });
  if (!result.success) process.exit(result.exitCode || 1);
}

function install(ref: string) {
  if (!existsSync(resolve(appDir, ".git"))) {
    run(["git", "clone", "--filter=blob:none", "--no-checkout", upstream, appDir]);
  }
  run(["git", "fetch", "--depth", "1", "origin", ref], appDir);
  run(["git", "checkout", "--detach", "--force", "FETCH_HEAD"], appDir);
  applyUssNavOverlay();
  // Upstream uses npm, but its Next.js application and lifecycle scripts run under Bun.
  // --ignore-scripts avoids the development-only Husky prepare hook.
  run(["bun", "install", "--ignore-scripts"], appDir);
}

if (command === "setup") {
  install(process.env.NEXT_AI_DRAWIO_REF ?? pinnedRef);
  console.log(`Next AI Draw.io is ready in ${appDir}`);
  process.exit(0);
}

if (command === "update") {
  install(process.env.NEXT_AI_DRAWIO_REF ?? "main");
  console.log("Next AI Draw.io updated. Run `bun run view` to start it.");
  process.exit(0);
}

if (command === "overlay") {
  if (!existsSync(resolve(appDir, ".git"))) {
    console.error("Run `bun run drawio:setup` first.");
    process.exit(1);
  }
  applyUssNavOverlay();
  console.log("USS-NAV Draw.io overlay applied.");
  process.exit(0);
}

if (command !== "dev") {
  console.error("Usage: bun tools/ai-drawio.ts [setup|dev|update|overlay]");
  process.exit(2);
}

if (!existsSync(resolve(appDir, "node_modules/.bin/next"))) {
  install(process.env.NEXT_AI_DRAWIO_REF ?? pinnedRef);
}
applyUssNavOverlay();

const url = `http://localhost:${port}`;
console.log(`Starting Next AI Draw.io at ${url}`);
const server = Bun.spawn(["bun", "x", "next", "dev", "--turbopack", "--port", port], {
  cwd: appDir,
  env: process.env,
  stdin: "inherit",
  stdout: "inherit",
  stderr: "inherit",
});

if (process.env.NO_OPEN !== "1") {
  void (async () => {
    for (let attempt = 0; attempt < 60; attempt++) {
      await Bun.sleep(500);
      try {
        const response = await fetch(url);
        if (!response.ok) continue;
        const opener = process.platform === "darwin" ? "open" : process.platform === "win32" ? "cmd" : "xdg-open";
        const args = process.platform === "win32" ? [opener, "/c", "start", url] : [opener, url];
        Bun.spawn(args, { stdout: "ignore", stderr: "ignore" }).unref();
        break;
      } catch {
        // The Next.js server is still compiling.
      }
    }
  })();
}

process.on("SIGINT", () => server.kill("SIGINT"));
process.on("SIGTERM", () => server.kill("SIGTERM"));
process.exit(await server.exited);
