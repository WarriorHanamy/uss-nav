export function parseYaml(text: string): Record<string, unknown> {
  const doc: Record<string, unknown> = {};
  let currentList: unknown[] = [];
  let currentObj: Record<string, unknown> | null = null;
  const stack: { key: string; container: unknown[] }[] = [];
  let listKey: string | null = null;

  const lines = text.split("\n");
  let i = 0;

  function getIndent(line: string): number {
    return line.search(/\S/);
  }

  function processLine(line: string, indent: number) {
    const trimmed = line.trim();
    if (trimmed === "" || trimmed.startsWith("#")) return;

    if (trimmed.startsWith("- ")) {
      const rest = trimmed.slice(2);
      if (!listKey) return;
      const arr = doc[listKey] as unknown[];
      if (rest.endsWith(":")) {
        const obj: Record<string, unknown> = {};
        arr.push(obj);
        currentObj = obj;
      } else {
        arr.push(rest);
      }
      return;
    }

    const colonIdx = trimmed.indexOf(":");
    if (colonIdx === -1) return;
    const key = trimmed.slice(0, colonIdx).trim();
    const hasValue = colonIdx < trimmed.length - 1 && trimmed[colonIdx + 1] === " ";

    if (hasValue) {
      let val: unknown = trimmed.slice(colonIdx + 2).trim();
      if (val === "true") val = true;
      else if (val === "false") val = false;
      else if (!isNaN(Number(val))) val = Number(val);

      if (currentObj) {
        currentObj[key] = val;
      } else {
        doc[key] = val;
      }
    } else {
      if (indent === 0) {
        listKey = key;
        doc[key] = [];
        currentObj = null;
      }
    }
  }

  for (const line of lines) {
    const indent = getIndent(line);
    processLine(line, indent);
  }

  return doc;
}
