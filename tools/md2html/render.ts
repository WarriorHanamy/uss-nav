import { readFileSync, writeFileSync, mkdirSync } from "fs";
import { resolve, dirname } from "path";
import { fileURLToPath } from "url";
import { execSync } from "child_process";
import { marked } from "marked";
import { markedHighlight } from "marked-highlight";
import hljs from "highlight.js";
import markedKatex from "marked-katex-extension";

const __dirname = dirname(fileURLToPath(import.meta.url));
const PROJECT_ROOT = resolve(__dirname, "../../");
const pkg = JSON.parse(readFileSync(resolve(__dirname, "package.json"), "utf-8"));
const cfg: Record<string, any> = pkg.md2html ?? {};

// ── configure marked ────────────────────────────────────────────────
marked.use(markedHighlight({
  langPrefix: "language-",
  highlight(code, lang) {
    const language = hljs.getLanguage(lang) ? lang : "plaintext";
    try {
      return hljs.highlight(code, { language }).value;
    } catch {
      return code;
    }
  },
}));

marked.use(markedKatex({
  throwOnError: false,
  nonStandard: true,
}));

// ── strip surrounding <p> from blockquotes ──────────────────────────
function stripSurroundingP(html: string): string {
  return html.replace(/<blockquote>\s*<p>(.*?)<\/p>\s*<\/blockquote>/gs, "<blockquote>$1</blockquote>");
}

// ── wrap tables in scrollable container ─────────────────────────────
function wrapTables(html: string): string {
  return html.replace(
    /<table>/g,
    '<div class="table-wrap"><table>',
  ).replace(
    /<\/table>/g,
    "</table></div>",
  );
}

// ── build TOC entries from raw markdown ─────────────────────────────
interface TocEntry {
  level: number;
  text: string;
  id: string;
}
function buildToc(md: string): TocEntry[] {
  const toc: TocEntry[] = [];
  const lines = md.split("\n");
  for (const line of lines) {
    const h2 = line.match(/^## (.+)/);
    const h3 = line.match(/^### (.+)/);
    if (h2) {
      const text = h2[1].replace(/[`*_]/g, "").trim();
      toc.push({ level: 2, text: decodeHtmlEntities(text), id: slugify(text) });
    } else if (h3) {
      const text = h3[1].replace(/[`*_]/g, "").trim();
      toc.push({ level: 3, text: decodeHtmlEntities(text), id: slugify(text) });
    }
  }
  return toc;
}

function decodeHtmlEntities(text: string): string {
  return text
    .replace(/&amp;/g, "&")
    .replace(/&lt;/g, "<")
    .replace(/&gt;/g, ">")
    .replace(/&quot;/g, '"')
    .replace(/&#39;/g, "'")
    .replace(/&#x27;/g, "'");
}

function slugify(text: string): string {
  return decodeHtmlEntities(text)
    .toLowerCase()
    .replace(/[^a-z0-9\u4e00-\u9fa5]+/g, "-")
    .replace(/^-+|-+$/g, "");
}

// ── inject ids into headings for TOC anchor links ───────────────────
function injectHeadingIds(html: string): string {
  return html.replace(
    /<h([23])>(.*?)<\/h([23])>/g,
    (_, level, inner, __level) => {
      const text = inner.replace(/<[^>]*>/g, "").trim();
      const id = slugify(text);
      return `<h${level} id="${id}">${inner}</h${level}>`;
    },
  );
}

// ── add copy button to code blocks ──────────────────────────────────
function addCodeCopyButtons(html: string): string {
  return html.replace(
    /<pre><code class="language-(\w+)">/g,
    '<pre><button class="copy-btn" onclick="copyCode(this)" title="Copy">📋</button><code class="language-$1">',
  );
}

// ── convert mermaid fenced blocks ───────────────────────────────────
function convertMermaidBlocks(html: string): string {
  return html.replace(
    /<pre><code class="language-mermaid">([\s\S]*?)<\/code><\/pre>/g,
    '<pre class="mermaid">$1</pre>',
  );
}

// ── open file in system browser ─────────────────────────────────────
function openInBrowser(path: string) {
  const platform = process.platform;
  const cmd = platform === "darwin" ? "open"
    : platform === "win32" ? "start"
    : "xdg-open";
  try {
    execSync(`${cmd} "${path}"`, { stdio: "ignore", timeout: 3000 });
    console.log("🌐 Opened in browser");
  } catch {
    console.log("⚠️  Could not auto-open browser, file is at:", path);
  }
}

// ── render a single markdown file to body HTML + TOC ──────────────
function renderMd(inputPath: string): { body: string; toc: TocEntry[] } {
  const md = readFileSync(inputPath, "utf-8");
  const toc = buildToc(md);
  let bodyHtml = marked.parse(md) as string;
  bodyHtml = stripSurroundingP(bodyHtml);
  bodyHtml = wrapTables(bodyHtml);
  bodyHtml = injectHeadingIds(bodyHtml);
  bodyHtml = convertMermaidBlocks(bodyHtml);
  bodyHtml = addCodeCopyButtons(bodyHtml);
  return { body: bodyHtml, toc };
}

// ── render: multi-tab view ────────────────────────────────────────────
function renderMulti(outputPath: string, tabs: { label: string; path: string }[]) {
  mkdirSync(dirname(outputPath), { recursive: true });

  // Render each document
  const results = tabs.map((t) => {
    const r = renderMd(t.path);
    return { ...r, label: t.label };
  });

  // Build tab panes HTML (pre-rendered, just shown/hidden by JS)
  const tabPanesHtml = results
    .map(
      (r, i) =>
        `<div class="tab-pane${i === 0 ? " active" : ""}" data-tab="${i}">${r.body}</div>`,
    )
    .join("\n");

  // Build JSON data for JS
  const tabsJson = JSON.stringify(
    tabs.map((t) => ({ label: t.label, id: t.label.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "") })),
  );
  const tocsJson = JSON.stringify(results.map((r) => r.toc));

  const templatePath = resolve(__dirname, "template_tabs.html");
  let template = readFileSync(templatePath, "utf-8");

  const title = "USS-NAV Architecture Overview";
  const desc = "Architecture documentation: EGO Planner, SceneGraph, and decision systems";
  const result = template
    .replace("{{TITLE}}", title)
    .replace("{{DESCRIPTION}}", desc)
    .replace("{{TABS}}", tabsJson)
    .replace("{{TOCS}}", tocsJson)
    .replace("{{TAB_PANES}}", tabPanesHtml);

  writeFileSync(outputPath, result, "utf-8");
  console.log(`✅ Generated ${outputPath} (${(result.length / 1024).toFixed(0)} KB)`);

  openInBrowser(outputPath);
}

// ── main ────────────────────────────────────────────────────────────
function main() {
  const args = process.argv.slice(2);

  // ── multi-tab mode ──
  if (args[0] === "--multi") {
    const outputPath = resolve(process.cwd(), args[1]);
    const tabs = args.slice(2).map((arg) => {
      const colonIdx = arg.indexOf(":");
      if (colonIdx === -1) throw new Error(`Invalid tab spec "${arg}" — expected label:path`);
      return { label: arg.slice(0, colonIdx), path: resolve(process.cwd(), arg.slice(colonIdx + 1)) };
    });
    renderMulti(outputPath, tabs);
    return;
  }

  // ── single-doc mode ──
  const inputPath = args[0]
    ? resolve(process.cwd(), args[0])
    : resolve(PROJECT_ROOT, cfg.input || "CODEBASE.md");
  const outputPath = args[1]
    ? resolve(process.cwd(), args[1])
    : resolve(PROJECT_ROOT, cfg.output || "_site/CODEBASE.html");

  mkdirSync(dirname(outputPath), { recursive: true });

  const { body, toc } = renderMd(inputPath);
  const tocJson = JSON.stringify(toc);

  const templatePath = resolve(__dirname, "template.html");
  let template = readFileSync(templatePath, "utf-8");

  const title = cfg.title || "CODEBASE Documentation";
  const desc = cfg.description || "Auto-generated repository documentation";
  const result = template
    .replace("{{TITLE}}", title)
    .replace("{{DESCRIPTION}}", desc)
    .replace("{{TOC}}", tocJson)
    .replace("{{BODY}}", body);

  writeFileSync(outputPath, result, "utf-8");
  console.log(`✅ Generated ${outputPath} (${(result.length / 1024).toFixed(0)} KB)`);

  if (cfg.autoOpen !== false) {
    openInBrowser(outputPath);
  }
}

main();
