---
name: drawio-web-render
description: Build, update, and verify USS-NAV multi-page Draw.io views served by the Bun-managed Next AI Draw.io integration. Use when editing tools/ai-drawio, tools/ai-drawio/uss-nav.drawio, adding diagram pages or bottom tabs, changing the Draw.io web launch flow, diagnosing Page-1 or session-restore bugs, or validating that repository architecture diagrams really render in the browser.
---

# Draw.io Web Render

Maintain the repository's editable multi-page Draw.io file and Next.js overlay. Treat browser-visible output, not valid XML or successful compilation, as completion.

## Hard UI constraints

- Do not show a shape/library sidebar or custom page-navigation sidebar.
- Collapse the host AI chat side panel to width `0` by default. Keep it user-toggleable with `Ctrl/Cmd+B`.
- Keep named page tabs visible along the bottom edge.
- Set `urlParameters.libraries` to `false`.
- Store every architecture view as a separate `<diagram name="...">` in one `<mxfile>`.
- Do not replace bottom tabs with buttons, dropdowns, side navigation, separate files, or separate routes.

Reject a change if its default browser view contains a sidebar or only shows `Page-1`.

## Repository contract

- Canonical diagram: `tools/ai-drawio/uss-nav.drawio`.
- Bun launcher and reproducible overlay: `tools/ai-drawio.ts`.
- Generated upstream checkout: `tools/.cache/next-ai-draw-io`; never treat edits there as source changes.
- Interactive command: `bun run drawio:view`.
- Documentation portal: `bun run view`; do not silently change it to Draw.io.

## Implementation workflow

1. Inspect the canonical XML and `applyUssNavOverlay()`.
2. Add pages to the canonical `<mxfile>` with unique cell IDs and descriptive page names.
3. Keep the overlay idempotent across a fresh pinned checkout and repeated runs.
4. Copy the canonical file into upstream `public/` only through the overlay.
5. Feed complete XML through `<DrawIoEmbed xml={...}>`; do not depend only on an immediate imperative load after empty `Page-1`.
6. Account for ChatPanel IndexedDB restoration. An empty session calls `clearDiagram()` and can overwrite initial XML. Reapply canonical XML once after draw.io reports ready.
7. Keep the reapply bounded so startup does not overwrite later user edits.
8. Keep `libraries: false`, collapse the host chat panel to `0`, and retain bottom page tabs.

## Required validation

Run structural checks:

```bash
xmllint --noout tools/ai-drawio/uss-nav.drawio
xmllint --xpath 'count(/mxfile/diagram)' tools/ai-drawio/uss-nav.drawio
bun build tools/ai-drawio.ts --target bun --outdir /tmp/uss-nav-ai-drawio-check
bun tools/ai-drawio.ts overlay
git diff --check
```

Then perform the mandatory visual check:

1. Stop stale Next.js processes for the cached upstream directory.
2. Start `bun run drawio:view`.
3. Open a normal GUI browser. Headless Chromium may fail to load `embed.diagrams.net` while the GUI browser works.
4. Wait for Draw.io and IndexedDB restoration.
5. Capture a GUI screenshot with `grim` or an equivalent tool.
6. Inspect the screenshot itself.

The screenshot must prove:

- The canvas contains USS-NAV content, not a loading screen or blank `Page-1`.
- Every expected page name appears in the bottom tab bar.
- The new or changed page is selected and rendered.
- No shape/library sidebar, page-navigation sidebar, or expanded host chat sidebar is visible.

Do not claim completion from XML parsing, HTTP 200, compilation, a served asset, or a headless screenshot stuck on “Draw.io panel is loading”.

## Known failure pattern

Without an `xml` prop, `react-drawio` initially sends an empty load. Separately, ChatPanel restores IndexedDB and clears a session with no real diagram. An imperative default load can therefore be overwritten and leave `Page-1`.

Use both defenses:

- Supply canonical XML through the component `xml` prop.
- Reapply it after the editor is ready to win over empty session restoration.

## Handoff

Report exact page names visually confirmed, the screenshot path used, and the user command. If the GUI iframe cannot load, report validation as incomplete and continue diagnosing.
