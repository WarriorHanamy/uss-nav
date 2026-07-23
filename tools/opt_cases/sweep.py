# /// script
# requires-python = ">=3.10"
# dependencies = ["pyyaml"]
# ///
"""Parallel parameter sweep over dumped SUPER optimization corner cases.

For every (case, param-combo) pair, runs the ROS-free opt_case_replay binary
inside one short-lived Docker container, then aggregates all results into
results.csv and a human-readable report.md.

Usage:
    uv run --script tools/opt_cases/sweep.py \
        --cases .artifacts/opt_cases \
        --grid tools/opt_cases/grid_example.yaml \
        --parallel 8 \
        --out .artifacts/sweeps/sweep-001

Grid yaml format (cartesian product):
    params:
      traj_opt/exp_traj/penna_t: [1000.0, 5000.0]
      traj_opt/exp_traj/penna_pos: [1.0e5, 5.0e5]
"""

import argparse
import concurrent.futures
import csv
import itertools
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

REPLAY_BIN = "/workspace/devel/lib/super_planner/opt_case_replay"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def path_to_nested(path: str, value) -> dict:
    keys = [k for k in path.split("/") if k]
    node = value
    for key in reversed(keys):
        node = {key: node}
    return node


def deep_merge(dst: dict, src: dict) -> dict:
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            deep_merge(dst[key], value)
        else:
            dst[key] = value
    return dst


def expand_grid(grid: dict) -> list[dict]:
    params = grid.get("params", {})
    names = list(params.keys())
    combos = []
    for values in itertools.product(*(params[n] for n in names)):
        override: dict = {}
        label_parts = []
        for name, value in zip(names, values):
            deep_merge(override, path_to_nested(name, value))
            label_parts.append(f"{name.split('/')[-1]}={value}")
        combos.append({"override": override, "label": ",".join(label_parts)})
    return combos


def find_cases(cases_path: Path) -> list[Path]:
    if (cases_path / "case.yaml").exists():
        return [cases_path]
    return sorted(p for p in cases_path.iterdir() if (p / "case.yaml").exists())


def run_task(
    task: dict, root: Path, image: str, regen_sfc: bool, repeat: int, cpuset: str
) -> dict:
    task_dir = Path(task["task_dir"])
    task_dir.mkdir(parents=True, exist_ok=True)
    override_path = task_dir / "override.yaml"
    override_path.write_text(yaml.safe_dump(task["combo"]["override"]))
    result_path = task_dir / "result.yaml"

    case_rel = Path(task["case_dir"]).resolve().relative_to(root)
    ovr_rel = override_path.resolve().relative_to(root)
    out_rel = result_path.resolve().relative_to(root)

    cmd = [
        "docker",
        "run",
        "--rm",
        "--name",
        task["container"],
        "--entrypoint",
        "bash",
    ]
    if cpuset:
        cmd += ["--cpuset-cpus", cpuset]
    cmd += [
        "-v",
        f"{root}/.artifacts/devel:/workspace/devel",
        "-v",
        f"{root}/.artifacts:/workspace/.artifacts",
        "-v",
        f"{root}/ws_main/src:/workspace/src:ro",
        image,
        "-c",
        f"source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && "
        f"{REPLAY_BIN} --case /workspace/{case_rel} --override /workspace/{ovr_rel} "
        f"--repeat {repeat} --out /workspace/{out_rel}"
        + (" --regen-sfc" if regen_sfc else ""),
    ]
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    wall = time.time() - t0

    result = {
        "case_id": task["case_id"],
        "combo_label": task["combo"]["label"],
        "exit_code": proc.returncode,
        "wall_time_s": round(wall, 3),
        "task_dir": str(task_dir),
    }
    if result_path.exists():
        try:
            loaded = yaml.safe_load(result_path.read_text()) or {}
            # Flatten nested backup section and drop raw time series from the row.
            backup = loaded.pop("backup", None) or {}
            loaded.pop("exp_opt_times", None)
            result.update(loaded)
            if backup:
                result["backup_success"] = backup.get("success")
                result["backup_opt_time"] = backup.get("opt_time")
        except yaml.YAMLError as exc:
            result["parse_error"] = str(exc)
    else:
        result["error"] = (proc.stderr or proc.stdout)[-500:]
    return result


def stats(values: list[float]) -> dict:
    if not values:
        return {}
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


def write_report(sweep_dir: Path, results: list[dict]) -> None:
    lines = ["# SUPER corner-case parameter sweep report", ""]
    by_case: dict[str, list[dict]] = {}
    for r in results:
        by_case.setdefault(r["case_id"], []).append(r)

    lines.append(f"- cases: {len(by_case)}")
    lines.append(f"- runs: {len(results)}")
    solved = sum(1 for r in results if r.get("ret") == 0)
    lines.append(f"- solved: {solved}/{len(results)}")
    lines.append("")

    # Per-combo aggregate across cases, ranked by success rate -> std -> mean.
    by_combo: dict[str, list[dict]] = {}
    for r in results:
        by_combo.setdefault(r["combo_label"], []).append(r)
    agg = []
    for label, runs in by_combo.items():
        ok = [r for r in runs if r.get("ret") == 0]
        entry = {
            "label": label,
            "success_rate": len(ok) / len(runs),
            "time": stats(
                [
                    r["opt_time"]
                    for r in ok
                    if isinstance(r.get("opt_time"), (int, float))
                ]
            ),
            "sfc": stats(
                [
                    r["sfc_time"]
                    for r in ok
                    if isinstance(r.get("sfc_time"), (int, float))
                ]
            ),
            "backup": stats(
                [
                    r["backup_opt_time"]
                    for r in ok
                    if isinstance(r.get("backup_opt_time"), (int, float))
                ]
            ),
        }
        agg.append(entry)
    agg.sort(
        key=lambda e: (
            -e["success_rate"],
            e["time"].get("std", float("inf")),
            e["time"].get("mean", float("inf")),
        )
    )

    lines.append("## Combo ranking (success rate -> opt_time std -> mean)")
    lines.append("")
    lines.append(
        "| combo | succ | opt mean [ms] | opt std [ms] | opt p90 [ms] | sfc mean [ms] | backup mean [ms] |"
    )
    lines.append("|---|---|---|---|---|---|---|")
    for e in agg:
        t, s, b = e["time"], e["sfc"], e["backup"]
        lines.append(
            f"| {e['label']} | {e['success_rate']:.0%} "
            f"| {t.get('mean', 0) * 1e3:.2f} | {t.get('std', 0) * 1e3:.2f} | {t.get('p90', 0) * 1e3:.2f} "
            f"| {s.get('mean', 0) * 1e3:.2f} | {b.get('mean', 0) * 1e3:.2f} |"
        )
    if agg:
        lines.append("")
        lines.append(f"**best overall: `{agg[0]['label']}`**")
    lines.append("")

    for case_id, runs in sorted(by_case.items()):
        reason = runs[0].get("reason", "?")
        lines.append(f"## {case_id} (reason={reason})")
        lines.append("")
        lines.append("| combo | ret | lbfgs_ret | iter | final_cost | opt_time [s] |")
        lines.append("|---|---|---|---|---|---|")
        runs_sorted = sorted(
            runs,
            key=lambda r: (r.get("ret", -1) != 0, r.get("final_cost", float("inf"))),
        )
        for r in runs_sorted:
            lines.append(
                f"| {r['combo_label']} | {r.get('ret', '?')} | {r.get('lbfgs_ret', '?')} "
                f"| {r.get('iter_num', '?')} | {r.get('final_cost', '?'):.4g} "
                f"| {r.get('opt_time', 0):.4f} |"
                if isinstance(r.get("final_cost"), (int, float))
                else f"| {r['combo_label']} | {r.get('ret', '?')} | - | - | - | - |"
            )
        best = next((r for r in runs_sorted if r.get("ret") == 0), None)
        lines.append("")
        lines.append(f"best: `{best['combo_label']}`" if best else "best: none solved")
        lines.append("")
    (sweep_dir / "report.md").write_text("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--cases",
        required=True,
        type=Path,
        help="case root dir (contains case-* dirs) or a single case dir",
    )
    parser.add_argument("--grid", required=True, type=Path, help="param grid yaml")
    parser.add_argument(
        "--parallel", type=int, default=max(1, (os.cpu_count() or 4) // 2)
    )
    parser.add_argument("--out", required=True, type=Path, help="sweep output dir")
    parser.add_argument("--image", default="ego-planner-sim")
    parser.add_argument(
        "--regen-sfc",
        action="store_true",
        help="regenerate SFC from dumped cloud instead of using dumped SFC",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=5,
        help="replays per task; opt_time is the best, all times recorded",
    )
    parser.add_argument(
        "--cpuset",
        default="",
        help="pin replay containers to these CPUs (e.g. 2-7) for "
        "low-noise timing; use with --parallel <= cpu count",
    )
    args = parser.parse_args()

    root = repo_root()
    args.cases = args.cases.resolve()
    args.out = args.out.resolve()
    cases = find_cases(args.cases)
    if not cases:
        print(f"no cases found under {args.cases}", file=sys.stderr)
        return 1
    combos = expand_grid(yaml.safe_load(args.grid.read_text()))
    if not combos:
        print("grid produced zero combinations", file=sys.stderr)
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    pid = os.getpid()
    tasks = []
    for case_dir in cases:
        case_yaml = yaml.safe_load((case_dir / "case.yaml").read_text())
        case_id = case_yaml.get("case_id", case_dir.name)
        for ci, combo in enumerate(combos):
            tasks.append(
                {
                    "case_dir": str(case_dir),
                    "case_id": case_id,
                    "combo": combo,
                    "task_dir": str(args.out / case_id / f"combo-{ci:03d}"),
                    "container": f"uss-nav-sweep-{pid}-{len(tasks)}",
                }
            )

    print(
        f"sweep: {len(cases)} cases x {len(combos)} combos = {len(tasks)} runs, "
        f"parallel={args.parallel}"
    )
    (args.out / "sweep_meta.json").write_text(
        json.dumps(
            {
                "cases": [str(c) for c in cases],
                "grid": str(args.grid),
                "combos": [c["label"] for c in combos],
                "regen_sfc": args.regen_sfc,
            },
            indent=2,
        )
    )

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel) as pool:
        futures = {
            pool.submit(
                run_task, t, root, args.image, args.regen_sfc, args.repeat, args.cpuset
            ): t
            for t in tasks
        }
        done = 0
        for fut in concurrent.futures.as_completed(futures):
            r = fut.result()
            results.append(r)
            done += 1
            print(
                f"[{done}/{len(tasks)}] {r['case_id']} {r['combo_label']} "
                f"-> ret={r.get('ret', 'ERR')}"
            )

    fieldnames = sorted({k for r in results for k in r.keys()})
    with open(args.out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    write_report(args.out, results)
    print(f"results: {args.out}/results.csv")
    print(f"report:  {args.out}/report.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
