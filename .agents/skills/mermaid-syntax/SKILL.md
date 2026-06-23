---
name: mermaid-syntax
description: Use when writing or editing Mermaid diagrams in markdown documentation, troubleshooting "Syntax error in text" browser errors, or validating diagrams before commit.
---

# Mermaid Syntax

## Overview

Mermaid v11.15.0 is used in this project's md2html documentation pipeline. The browser runtime (CDN `mermaid@11`) has stricter parsing than `mermaid.parse()` with `suppressErrors: true` — always validate with full rendering.

## Sequence Diagram Rules

- **No `--` inline comments** — `--` is a reserved token. `FSM->>SG: doThing()  -- comment` is a syntax error.
- **Participant aliases with spaces** must use `participant X as "Alias With Space"`. Unquoted spaces after `as` may parse inconsistently across versions.
- **Diagram must end cleanly** — no trailing `--`, `---`, or stray punctuation after the last message line.

## Flowchart Rules

- Use `flowchart` keyword (not deprecated `graph`).
- `direction TB` inside subgraphs is valid only with `flowchart` (not `graph`).
- Link text with special characters (`/`, `+`, `&`, `→`) must be in pipes with quotes: `--> |"text with /slashes/"| Node`.

## Validation

The `mermaid.parse()` API is permissive and false-negatives. Always validate with full browser rendering:

```bash
# Requires puppeteer (installed via @mermaid-js/mermaid-cli)
npx mmdc -i diagram.mmd -o /dev/null
```

For automated testing of all diagrams in a markdown file, extract each ` ```mermaid ` block and run mmdc individually.

## Common Mistakes

| Mistake | Error message | Fix |
|---------|---------------|-----|
| `-- comment` after message | `Expecting 'SPACE', 'NEWLINE', ..., got '-'` | Remove the `--` comment |
| `---` inside/after diagram | `Diagrams beginning with --- are not valid` | Remove extraneous `---` |
| `mermaid.parse()` pass but browser fails | Silent false-negative | Use `mmdc` instead |
| `graph` instead of `flowchart` | Works in v11 but deprecated | Use `flowchart` |
