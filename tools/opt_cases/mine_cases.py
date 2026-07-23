# /// script
# requires-python = ">=3.10"
# dependencies = ["pyyaml"]
# ///
"""Mine dumped SUPER corner cases from a trace directory.

Correlates the Fluent Bit machine-readable event stream (opt_events.jsonl,
falling back to fluentbit_roslog.log) with the on-disk case directories under
.artifacts/opt_cases/ and emits a dataset manifest (JSONL) plus a summary.

Usage:
    uv run --script tools/opt_cases/mine_cases.py \
        --trace .artifacts/traces/super-20260722-101530 \
        --cases-root .artifacts/opt_cases \
        --out .artifacts/opt_cases/manifest-super-20260722-101530.jsonl
"""

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

import yaml

DIFFICULTY_EVENTS = (
    "case_dumped",
    "exp_opt_failed",
    "sfc_failed",
    "path_search_failed",
    "replan_failed",
    "stuck_suspect",
)

TIMING_FIELDS = (
    "total_t",
    "frontend_t",
    "exp_sfc_t",
    "exp_opt_t",
    "back_frontend_t",
    "back_sfc_t",
    "back_opt_t",
    "viz_t",
)


def timing_stats(values: list[float]) -> dict:
    srt = sorted(values)
    n = len(srt)
    mean = sum(srt) / n
    var = sum((v - mean) ** 2 for v in srt) / n
    pick = lambda q: srt[min(n - 1, int(n * q))]
    return {
        "n": n,
        "mean": mean,
        "std": var**0.5,
        "p50": pick(0.5),
        "p90": pick(0.9),
        "p99": pick(0.99),
        "max": srt[-1],
    }


def print_timing_report(events: list[dict]) -> None:
    timing = [e for e in events if e.get("event") == "replan_timing"]
    if not timing:
        return
    print(f"\nreplan_timing breakdown over {len(timing)} replans (ms):")
    header = (
        f"{'field':<18}{'mean':>9}{'std':>9}{'p50':>9}{'p90':>9}{'p99':>9}{'max':>9}"
    )
    print(header)
    print("-" * len(header))
    for field in TIMING_FIELDS:
        values = [
            float(e[field]) * 1e3
            for e in timing
            if isinstance(e.get(field), (int, float))
        ]
        if not values:
            continue
        s = timing_stats(values)
        print(
            f"{field:<18}{s['mean']:>9.2f}{s['std']:>9.2f}{s['p50']:>9.2f}"
            f"{s['p90']:>9.2f}{s['p99']:>9.2f}{s['max']:>9.2f}"
        )


def parse_ltsv_line(line: str) -> dict:
    record = {}
    for field in line.split("\t"):
        key, sep, value = field.partition(":")
        if not sep:
            continue
        key = key.strip().strip('"')
        value = value.strip()
        if value.startswith('"'):
            try:
                value = json.loads(value)
            except json.JSONDecodeError:
                value = value.strip('"')
        else:
            try:
                value = int(value)
            except ValueError:
                try:
                    value = float(value)
                except ValueError:
                    pass
        record[key] = value
    return record


def load_events(trace_dir: Path) -> list[dict]:
    ltsv = trace_dir / "opt_events.ltsv"
    if ltsv.exists():
        return [
            parse_ltsv_line(line)
            for line in ltsv.read_text().splitlines()
            if line.strip()
        ]

    jsonl = trace_dir / "opt_events.jsonl"
    if jsonl.exists():
        events = []
        for line in jsonl.read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        return events

    # Fallback: parse the key=value text stream from fluent-bit.
    text_log = trace_dir / "fluentbit_roslog.log"
    events = []
    if not text_log.exists():
        return events
    kv_re = re.compile(r"([\w]+)=([^\s]+)")
    for line in text_log.read_text().splitlines():
        m = re.search(r"message=(.*)$", line)
        if not m:
            continue
        message = m.group(1)
        head = re.match(r"\s*-*\s*\[([\w]+)\](?:\[([\w]+)\])?\s*(.*)$", message)
        if not head:
            continue
        record = {"module": head.group(1), "message": message}
        if head.group(2):
            record["subtag"] = head.group(2)
        record.update(dict(kv_re.findall(head.group(3))))
        events.append(record)
    return events


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--trace", required=True, type=Path, help="trace directory")
    parser.add_argument("--cases-root", type=Path, default=Path(".artifacts/opt_cases"))
    parser.add_argument("--out", type=Path, help="manifest output path (JSONL)")
    args = parser.parse_args()

    events = load_events(args.trace)
    if not events:
        print(
            f"no events found in {args.trace} (opt_events.jsonl / fluentbit_roslog.log)",
            file=sys.stderr,
        )
        return 1

    counts = Counter(
        e.get("event") for e in events if e.get("event") in DIFFICULTY_EVENTS
    )
    print(f"trace: {args.trace}")
    for name in DIFFICULTY_EVENTS:
        if counts.get(name):
            print(f"  {name}: {counts[name]}")
    print_timing_report(events)

    dumped = [e for e in events if e.get("event") == "case_dumped"]
    manifest = []
    missing = 0
    for e in dumped:
        case_id = e.get("case_id", "")
        case_dir = args.cases_root / case_id
        entry = {
            "case_id": case_id,
            "trace_id": e.get("trace_id", args.trace.name),
            "reason": e.get("reason", ""),
            "stage": e.get("stage", ""),
            "time": e.get("time", e.get("date", "")),
            "case_dir": str(case_dir),
            "exists": (case_dir / "case.yaml").exists(),
        }
        if entry["exists"]:
            case_yaml = yaml.safe_load((case_dir / "case.yaml").read_text())
            entry["sfc_count"] = len(case_yaml.get("sfc", []))
            entry["has_cloud"] = (case_dir / "cloud.pcd").exists()
            result = case_yaml.get("result", {})
            entry["opt_success"] = result.get("opt_success")
            entry["iter_num"] = result.get("iter_num")
            entry["final_cost"] = result.get("final_cost")
        else:
            missing += 1
        manifest.append(entry)

    out = args.out or (args.cases_root / f"manifest-{args.trace.name}.jsonl")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "\n".join(json.dumps(e) for e in manifest) + ("\n" if manifest else "")
    )
    print(
        f"\ncases dumped: {len(manifest)} (on disk: {len(manifest) - missing}, "
        f"missing: {missing})"
    )
    print(f"manifest: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
