#!/usr/bin/env python3
"""Aggregate exploration benchmark trial outputs.

Expected layout:

    <root>/<env>/<planner>/trial_<n>/
        robot_a.json
        robot_b.json
        metrics.csv or exploration_*.csv

The per-robot JSON files come from ``session_reporter.py``.  The CSV file comes
from ``exploration_metrics_logger.py`` and carries global merged-map coverage.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROBOTS = ("robot_a", "robot_b")


@dataclass(frozen=True)
class TrialRecord:
    env: str
    planner: str
    trial: str
    path: Path
    robots: dict[str, dict[str, Any]]
    global_metrics: dict[str, float]


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _find_metrics_csv(trial_dir: Path) -> Path | None:
    exact = trial_dir / "metrics.csv"
    if exact.exists():
        return exact
    matches = sorted(trial_dir.glob("exploration_*.csv"))
    return matches[-1] if matches else None


def _load_last_csv_row(path: Path | None) -> dict[str, float]:
    if path is None or not path.exists():
        return {}
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    last = rows[-1]
    return {k: _float(v) for k, v in last.items() if k}


def collect_trial_records(root: Path) -> list[TrialRecord]:
    root = Path(root)
    records: list[TrialRecord] = []
    for trial_dir in sorted(root.glob("*/*/trial_*")):
        if not trial_dir.is_dir():
            continue
        rel = trial_dir.relative_to(root)
        if len(rel.parts) < 3:
            continue
        env, planner, trial = rel.parts[:3]
        robots = {ns: _load_json(trial_dir / f"{ns}.json") for ns in ROBOTS}
        global_metrics = _load_last_csv_row(_find_metrics_csv(trial_dir))
        records.append(TrialRecord(
            env=env,
            planner=planner,
            trial=trial,
            path=trial_dir,
            robots=robots,
            global_metrics=global_metrics,
        ))
    return records


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def _stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def _robot_value(record: TrialRecord, ns: str, section: str, key: str) -> float:
    data = record.robots.get(ns, {})
    return _float(data.get(section, {}).get(key))


def summarise_records(records: list[TrialRecord]) -> dict[tuple[str, str], dict[str, float]]:
    grouped: dict[tuple[str, str], list[TrialRecord]] = {}
    for record in records:
        grouped.setdefault((record.env, record.planner), []).append(record)

    summary: dict[tuple[str, str], dict[str, float]] = {}
    for key, group in sorted(grouped.items()):
        global_area = [
            _float(r.global_metrics.get("global_explored_area_m2"))
            for r in group
            if "global_explored_area_m2" in r.global_metrics
        ]
        global_cov = [
            _float(r.global_metrics.get("global_coverage_ratio"))
            for r in group
            if "global_coverage_ratio" in r.global_metrics
        ]
        overlap = [
            _float(r.global_metrics.get("overlap_pct"))
            for r in group
            if "overlap_pct" in r.global_metrics
        ]
        entry: dict[str, float] = {
            "trial_count": float(len(group)),
            "global_explored_area_m2_mean": _mean(global_area),
            "global_explored_area_m2_std": _stdev(global_area),
            "global_coverage_ratio_mean": _mean(global_cov),
            "global_coverage_ratio_std": _stdev(global_cov),
            "overlap_pct_mean": _mean(overlap),
            "overlap_pct_std": _stdev(overlap),
        }
        for ns in ROBOTS:
            distances = [_robot_value(r, ns, "progress", "distance_travelled_m") for r in group]
            areas = [_robot_value(r, ns, "coverage", "explored_area_m2") for r in group]
            contacts = [_robot_value(r, ns, "safety", "wall_contact_count") for r in group]
            entry[f"{ns}_distance_m_mean"] = _mean(distances)
            entry[f"{ns}_distance_m_std"] = _stdev(distances)
            entry[f"{ns}_explored_area_m2_mean"] = _mean(areas)
            entry[f"{ns}_explored_area_m2_std"] = _stdev(areas)
            entry[f"{ns}_wall_contacts_mean"] = _mean(contacts)
        summary[key] = entry
    return summary


def write_summary(root: Path, summary: dict[tuple[str, str], dict[str, float]]) -> tuple[Path, Path]:
    json_path = root / "summary.json"
    csv_path = root / "summary.csv"

    json_payload = {
        f"{env}/{planner}": metrics
        for (env, planner), metrics in summary.items()
    }
    json_path.write_text(json.dumps(json_payload, indent=2) + "\n")

    fieldnames = ["env", "planner"]
    for metrics in summary.values():
        for key in metrics:
            if key not in fieldnames:
                fieldnames.append(key)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for (env, planner), metrics in summary.items():
            row = {"env": env, "planner": planner}
            row.update(metrics)
            writer.writerow(row)
    return json_path, csv_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Benchmark output root.")
    args = parser.parse_args()

    records = collect_trial_records(args.root)
    summary = summarise_records(records)
    json_path, csv_path = write_summary(args.root, summary)
    print(f"records={len(records)} summary_json={json_path} summary_csv={csv_path}")
    return 0 if records else 1


if __name__ == "__main__":
    raise SystemExit(main())
