import { fileURLToPath } from "url";
import { dirname, resolve } from "path";

export function getRepoRoot(): string {
  const __filename = fileURLToPath(import.meta.url);
  return resolve(dirname(__filename), "../..");
}
