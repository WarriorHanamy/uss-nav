#!/usr/bin/env bun
/**
 * Doxygen HTML render wrapper.
 *
 * Calls system doxygen with the project Doxyfile and
 * opens the generated index.html in the default browser.
 */

import { $ } from "bun";
import { existsSync } from "fs";
import { join } from "path";

const PROJECT_ROOT = join(import.meta.dir, "../..");
const DOXYFILE = join(PROJECT_ROOT, "Doxyfile");
const OUTPUT_DIR = join(PROJECT_ROOT, "docs/api/html");
const INDEX_HTML = join(OUTPUT_DIR, "index.html");

async function main() {
  if (!existsSync(DOXYFILE)) {
    console.error("Doxyfile not found at", DOXYFILE);
    process.exit(1);
  }

  await $`mkdir -p ${OUTPUT_DIR}`;

  console.log("Generating Doxygen documentation...");
  const result = await $`doxygen ${DOXYFILE}`.cwd(PROJECT_ROOT);

  if (result.exitCode !== 0) {
    console.error("Doxygen failed with code", result.exitCode);
    process.exit(result.exitCode);
  }

  console.log("Documentation generated at", OUTPUT_DIR);

  if (existsSync(INDEX_HTML)) {
    await $`xdg-open ${INDEX_HTML}`;
  } else {
    console.warn("index.html not found at", INDEX_HTML);
  }
}

main();
