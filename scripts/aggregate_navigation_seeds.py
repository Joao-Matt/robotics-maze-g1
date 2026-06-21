#!/usr/bin/env python3
"""Aggregate navigation seed summaries into a solve-rate report."""

from __future__ import annotations

import argparse
import csv
from html import escape
import json
import math
from pathlib import Path
from typing import Any


DEFAULT_HELDOUT_SEEDS = [
    1126362096,
    1979650228,
    1206536813,
    795378426,
    711116612,
    1738064285,
    971229033,
    329188623,
    894390399,
    32784551,
    170038683,
    1285796317,
    299369674,
    380511688,
    1312910544,
    1648306726,
    1106423062,
    1945965940,
    916063502,
    781626286,
]


def parse_seed_values(values: list[str] | None) -> list[int]:
    if not values:
        return []
    seeds: list[int] = []
    for value in values:
        for item in value.replace(",", " ").split():
            seeds.append(int(item))
    return seeds


def wilson_interval(successes: int, total: int, z: float = 1.96) -> dict[str, float | None]:
    if total <= 0:
        return {"low": None, "high": None}
    proportion = successes / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    half_width = z * math.sqrt((proportion * (1.0 - proportion) + z * z / (4.0 * total)) / total) / denominator
    return {"low": max(0.0, center - half_width), "high": min(1.0, center + half_width)}


def numeric(value: object) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, q)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def median(values: list[float]) -> float | None:
    return percentile(values, 0.5)


def values_for(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = numeric(row.get(key))
        if value is not None:
            values.append(value)
    return values


def schema_completeness(summary: dict[str, Any]) -> float:
    checks = [
        "final_status" in summary,
        "maze_goal_reached" in summary,
        "physical_goal_error_m" in summary,
        "distance_traveled_m" in summary,
        "sim_elapsed_s" in summary,
        "wall_elapsed_s" in summary,
        "realtime_factor" in summary,
        "mapping" in summary and isinstance(summary.get("mapping"), dict),
        "motion" in summary and isinstance(summary.get("motion"), dict),
        "samples" in summary and isinstance(summary.get("samples"), dict),
        "localization_evaluation_ground_truth_only" in summary,
    ]
    return sum(1 for value in checks if value) / len(checks)


def failure_taxonomy(row: dict[str, Any]) -> str:
    if row.get("success"):
        return "success"
    status = str(row.get("final_status") or row.get("status") or "").upper()
    if "COLLISION" in status or numeric(row.get("wall_contact_count") or 0.0) not in (None, 0.0):
        return "hit"
    if "FALL" in status:
        return "fall"
    if "STUCK" in status:
        return "stuck"
    if "LOST" in status or "ODOMETRY" in status or "SCAN" in status:
        return "lost"
    if "TIMEOUT" in status or status == "RUNNING":
        return "timeout"
    if "NAV2" in status or "ABORT" in status or "CANCEL" in status:
        return "nav_abort"
    if "PHYSICAL" in status or "DISAGREEMENT" in status:
        return "goal_error"
    if not status:
        return "unknown"
    return status.lower()


def latest_summary_paths(root: Path, seeds: list[int] | None = None) -> dict[int, Path]:
    expected = set(seeds or [])
    latest: dict[int, Path] = {}
    for path in root.glob("seed-*/*/summary.json"):
        if path.parent.name == "latest":
            continue
        seed_name = path.parent.parent.name
        if not seed_name.startswith("seed-"):
            continue
        try:
            seed = int(seed_name.removeprefix("seed-"))
        except ValueError:
            continue
        if expected and seed not in expected:
            continue
        current = latest.get(seed)
        if current is None or path.parent.name > current.parent.name:
            latest[seed] = path
    return latest


def summarize(
    root: Path,
    *,
    label: str,
    seeds: list[int] | None = None,
    success_field: str = "maze_goal_reached",
) -> dict[str, Any]:
    paths = latest_summary_paths(root, seeds)
    expected = list(seeds or [])
    rows = []
    for seed in sorted(paths):
        path = paths[seed]
        summary = json.loads(path.read_text(encoding="utf-8"))
        mapping = summary.get("mapping", {})
        motion = summary.get("motion", {})
        samples = summary.get("samples", {})
        localization = summary.get("localization_evaluation_ground_truth_only", {})
        contact_counts = motion.get("contact_counts", {}) if isinstance(motion, dict) else {}
        recovery_events = motion.get("recovery_events", []) if isinstance(motion, dict) else []
        sim_elapsed = numeric(summary.get("sim_elapsed_s"))
        nav_odom_samples = samples.get("navigation_odom") if isinstance(samples, dict) else None
        gt_samples = samples.get("ground_truth_evaluation") if isinstance(samples, dict) else None
        nav2_commands = samples.get("nav2_commands") if isinstance(samples, dict) else None
        rows.append(
            {
                "seed": seed,
                "success": bool(summary.get(success_field)),
                "success_field": success_field,
                "final_status": summary.get("final_status"),
                "status": summary.get("status"),
                "physical_goal_error_m": summary.get("physical_goal_error_m"),
                "best_path_completion_fraction": summary.get("best_path_completion_fraction"),
                "final_path_completion_fraction": summary.get("final_path_completion_fraction"),
                "path_efficiency": summary.get("path_efficiency"),
                "remaining_path_distance_m": summary.get("remaining_path_distance_m"),
                "distance_traveled_m": summary.get("distance_traveled_m"),
                "known_cells_final": summary.get("known_cells_final"),
                "coverage_fraction": mapping.get("coverage_fraction") if isinstance(mapping, dict) else None,
                "wall_contact_count": contact_counts.get("wall") if isinstance(contact_counts, dict) else None,
                "recovery_event_count": len(recovery_events) if isinstance(recovery_events, list) else None,
                "localization_rmse_m": localization.get("position_rmse_m") if isinstance(localization, dict) else None,
                "localization_final_error_m": localization.get("final_position_error_m") if isinstance(localization, dict) else None,
                "localization_drift_scale": localization.get("distance_scale") if isinstance(localization, dict) else None,
                "localization_aligned_samples": localization.get("aligned_samples") if isinstance(localization, dict) else None,
                "sim_elapsed_s": summary.get("sim_elapsed_s"),
                "wall_elapsed_s": summary.get("wall_elapsed_s"),
                "realtime_factor": summary.get("realtime_factor"),
                "navigation_odom_rate_hz": nav_odom_samples / sim_elapsed if sim_elapsed and isinstance(nav_odom_samples, (int, float)) else None,
                "ground_truth_eval_rate_hz": gt_samples / sim_elapsed if sim_elapsed and isinstance(gt_samples, (int, float)) else None,
                "nav2_command_rate_hz": nav2_commands / sim_elapsed if sim_elapsed and isinstance(nav2_commands, (int, float)) else None,
                "schema_completeness_fraction": schema_completeness(summary),
                "run_dir": str(path.parent),
                "summary_path": str(path),
            }
        )
        rows[-1]["failure_taxonomy"] = failure_taxonomy(rows[-1])

    total = len(rows)
    successes = sum(1 for row in rows if row["success"])
    solve_rate = successes / total if total else None
    missing = sorted(set(expected) - set(paths)) if expected else []
    return {
        "schema_version": 1,
        "label": label,
        "root": str(root),
        "success_field": success_field,
        "expected_count": len(expected) if expected else None,
        "count": total,
        "goal_reached_count": successes,
        "solve_rate": solve_rate,
        "wilson_95_ci": wilson_interval(successes, total),
        "missing_seeds": missing,
        "results": rows,
    }


def aggregate_kpis(report: dict[str, Any], compare: dict[str, Any] | None = None) -> dict[str, Any]:
    rows = list(report["results"])
    failed = [row for row in rows if not row.get("success")]
    successes = [row for row in rows if row.get("success")]
    taxonomy_counts: dict[str, int] = {}
    for row in rows:
        key = str(row.get("failure_taxonomy") or failure_taxonomy(row))
        taxonomy_counts[key] = taxonomy_counts.get(key, 0) + 1
    failure_counts = {key: value for key, value in taxonomy_counts.items() if key != "success"}
    total_failures = sum(failure_counts.values())
    wall_contact_runs = sum(1 for row in rows if (numeric(row.get("wall_contact_count")) or 0.0) > 0.0)
    total_wall_contacts = sum(numeric(row.get("wall_contact_count")) or 0.0 for row in rows)
    distance = sum(numeric(row.get("distance_traveled_m")) or 0.0 for row in rows)
    solve_rate = report.get("solve_rate")
    compare_rate = compare.get("solve_rate") if compare else None
    overfit_gap = compare_rate - solve_rate if isinstance(compare_rate, (int, float)) and isinstance(solve_rate, (int, float)) else None
    return {
        "success": {
            "solve_rate": solve_rate,
            "goal_reached_count": report.get("goal_reached_count"),
            "count": report.get("count"),
            "expected_count": report.get("expected_count"),
            "missing_count": len(report.get("missing_seeds", [])),
            "wilson_95_ci": report.get("wilson_95_ci"),
            "seen_vs_heldout_gap": overfit_gap,
        },
        "mission": {
            "path_efficiency_median": median(values_for(rows, "path_efficiency")),
            "path_efficiency_p95": percentile(values_for(rows, "path_efficiency"), 0.95),
            "time_to_goal_median_sim_s": median(values_for(successes, "sim_elapsed_s")),
            "final_goal_error_median_m": median(values_for(rows, "physical_goal_error_m")),
            "final_goal_error_p95_m": percentile(values_for(rows, "physical_goal_error_m"), 0.95),
            "distance_median_m": median(values_for(rows, "distance_traveled_m")),
        },
        "safety_motion": {
            "wall_contact_runs": wall_contact_runs,
            "total_wall_contacts": total_wall_contacts,
            "recovery_events_total": sum(numeric(row.get("recovery_event_count")) or 0.0 for row in rows),
            "clean_motion_rate": 1.0 - wall_contact_runs / len(rows) if rows else None,
        },
        "localization": {
            "ate_rmse_median_m": median(values_for(rows, "localization_rmse_m")),
            "ate_rmse_p95_m": percentile(values_for(rows, "localization_rmse_m"), 0.95),
            "final_error_median_m": median(values_for(rows, "localization_final_error_m")),
            "drift_scale_median": median(values_for(rows, "localization_drift_scale")),
        },
        "mapping": {
            "coverage_median": median(values_for(rows, "coverage_fraction")),
            "coverage_p05": percentile(values_for(rows, "coverage_fraction"), 0.05),
            "known_cells_median": median(values_for(rows, "known_cells_final")),
        },
        "data_quality": {
            "navigation_odom_rate_median_hz": median(values_for(rows, "navigation_odom_rate_hz")),
            "ground_truth_eval_rate_median_hz": median(values_for(rows, "ground_truth_eval_rate_hz")),
            "nav2_command_rate_median_hz": median(values_for(rows, "nav2_command_rate_hz")),
            "schema_completeness_min": min(values_for(rows, "schema_completeness_fraction"), default=None),
            "realtime_factor_median": median(values_for(rows, "realtime_factor")),
            "realtime_factor_min": min(values_for(rows, "realtime_factor"), default=None),
        },
        "reliability": {
            "mtbf_m_per_failure": distance / total_failures if total_failures else None,
            "failure_taxonomy": failure_counts,
            "dominant_failure_modes": sorted(failure_counts.items(), key=lambda item: (-item[1], item[0]))[:3],
            "crash_safe_summary_rate": sum(1 for row in rows if row.get("final_status") and row.get("final_status") != "RUNNING") / len(rows) if rows else None,
        },
    }


def verdict(kpis: dict[str, Any]) -> dict[str, str]:
    success = kpis["success"]
    safety = kpis["safety_motion"]
    localization = kpis["localization"]
    data_quality = kpis["data_quality"]
    solve_rate = numeric(success.get("solve_rate"))
    missing_count = int(success.get("missing_count") or 0)
    wall_contact_runs = int(safety.get("wall_contact_runs") or 0)
    ate_rmse = numeric(localization.get("ate_rmse_median_m"))
    rtf = numeric(data_quality.get("realtime_factor_median"))
    schema_min = numeric(data_quality.get("schema_completeness_min"))
    if missing_count:
        return {
            "label": "No - evidence incomplete",
            "class": "bad",
            "text": f"{missing_count} held-out seeds are missing, so the report cannot support an unsupervised trust claim.",
        }
    if solve_rate is None:
        return {"label": "No - no held-out runs", "class": "bad", "text": "No held-out summaries were found."}
    if solve_rate >= 0.9 and wall_contact_runs == 0 and (ate_rmse is None or ate_rmse <= 0.75) and (rtf is None or rtf >= 0.8):
        return {
            "label": "Qualified yes",
            "class": "good",
            "text": "Held-out success, clean motion, localization, and realtime behavior are strong enough for supervised rollout toward unsupervised operation.",
        }
    if solve_rate >= 0.75 and wall_contact_runs <= 1 and (schema_min is None or schema_min >= 0.8):
        return {
            "label": "Not yet - supervised only",
            "class": "warn",
            "text": "The robot shows promise, but residual failures mean it should run with supervision and abort/recovery monitoring.",
        }
    return {
        "label": "No",
        "class": "bad",
        "text": "Held-out reliability or safety is not strong enough to trust the robot to run unsupervised.",
    }


def write_report_files(
    report: dict[str, Any],
    json_path: Path | None,
    csv_path: Path | None,
    html_path: Path | None = None,
    compare: dict[str, Any] | None = None,
) -> None:
    kpis = aggregate_kpis(report, compare)
    report["kpis"] = kpis
    report["verdict"] = verdict(kpis)
    if compare is not None:
        compare_kpis = aggregate_kpis(compare)
        report["comparison"] = {
            "label": compare["label"],
            "count": compare["count"],
            "goal_reached_count": compare["goal_reached_count"],
            "solve_rate": compare["solve_rate"],
            "wilson_95_ci": compare["wilson_95_ci"],
            "seen_vs_heldout_gap": kpis["success"]["seen_vs_heldout_gap"],
            "kpis": compare_kpis,
        }
    if json_path:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if csv_path:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        rows = report["results"]
        fieldnames = [
            "seed",
            "success",
            "final_status",
            "physical_goal_error_m",
            "best_path_completion_fraction",
            "final_path_completion_fraction",
            "distance_traveled_m",
            "known_cells_final",
            "coverage_fraction",
            "wall_contact_count",
            "recovery_event_count",
            "failure_taxonomy",
            "path_efficiency",
            "remaining_path_distance_m",
            "localization_rmse_m",
            "localization_final_error_m",
            "localization_drift_scale",
            "realtime_factor",
            "navigation_odom_rate_hz",
            "nav2_command_rate_hz",
            "schema_completeness_fraction",
            "sim_elapsed_s",
            "wall_elapsed_s",
            "run_dir",
            "summary_path",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
    if html_path:
        html_path.parent.mkdir(parents=True, exist_ok=True)
        html_path.write_text(render_html_report(report, compare), encoding="utf-8")


def format_number(value: object, digits: int = 2) -> str:
    number = numeric(value)
    if number is None:
        return "n/a"
    if abs(number) >= 100:
        return f"{number:.0f}"
    return f"{number:.{digits}f}"


def format_metric(value: object, unit: str = "") -> str:
    if unit == "%":
        return format_percent(numeric(value))
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, (int, float)):
        number = numeric(value)
        if number is None:
            return "n/a"
        suffix = f" {unit}" if unit else ""
        return f"{format_number(number)}{suffix}"
    return str(value)


def metric_block(title: str, subtitle: str, rows: list[tuple[str, object, str]]) -> str:
    items = []
    for label, value, unit in rows:
        items.append(
            f"<div class='metric-row'><span>{escape(label)}</span><b>{escape(format_metric(value, unit))}</b></div>"
        )
    return (
        "<section class='metric-block'>"
        f"<p class='eyebrow'>{escape(title)}</p><h3>{escape(subtitle)}</h3>"
        f"<div class='metric-rows'>{''.join(items)}</div></section>"
    )


def svg_bar_chart(
    title: str,
    labels: list[str],
    values: list[float],
    *,
    colors: list[str] | None = None,
    max_value: float | None = None,
    value_suffix: str = "",
) -> str:
    if not values:
        return f"<section class='plot'><h3>{escape(title)}</h3><p class='muted'>No data available.</p></section>"
    width, height = 760, 260
    margin_left, margin_top, margin_bottom = 52, 34, 48
    plot_w, plot_h = width - margin_left - 18, height - margin_top - margin_bottom
    ceiling = max_value if max_value is not None else max(values) or 1.0
    ceiling = max(1e-9, ceiling)
    bar_gap = 4
    bar_w = max(3, (plot_w - bar_gap * (len(values) - 1)) / len(values))
    parts = [
        f"<section class='plot'><h3>{escape(title)}</h3><svg viewBox='0 0 {width} {height}' role='img'>",
        "<rect width='100%' height='100%' fill='white'/>",
        f"<line x1='{margin_left}' y1='{margin_top + plot_h}' x2='{width - 12}' y2='{margin_top + plot_h}' stroke='#d9dee7'/>",
        f"<line x1='{margin_left}' y1='{margin_top}' x2='{margin_left}' y2='{margin_top + plot_h}' stroke='#d9dee7'/>",
    ]
    for index, value in enumerate(values):
        bar_h = max(1.0, plot_h * max(0.0, value) / ceiling)
        x = margin_left + index * (bar_w + bar_gap)
        y = margin_top + plot_h - bar_h
        color = colors[index] if colors else "#0f766e"
        label = labels[index]
        parts.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{bar_w:.1f}' height='{bar_h:.1f}' fill='{color}' rx='2'/>")
        if len(values) <= 24:
            parts.append(f"<text x='{x + bar_w / 2:.1f}' y='{height - 18}' text-anchor='middle' font-size='10' fill='#6b7280'>{escape(label)}</text>")
        parts.append(f"<title>{escape(label)}: {format_number(value)}{escape(value_suffix)}</title>")
    parts.append(f"<text x='{margin_left}' y='20' font-size='12' fill='#6b7280'>max {format_number(ceiling)}{escape(value_suffix)}</text>")
    parts.append("</svg></section>")
    return "".join(parts)


def svg_histogram(title: str, values: list[float], *, bins: int = 8, suffix: str = "") -> str:
    if not values:
        return f"<section class='plot'><h3>{escape(title)}</h3><p class='muted'>No data available.</p></section>"
    low, high = min(values), max(values)
    if math.isclose(low, high):
        labels = [format_number(low)]
        counts = [len(values)]
    else:
        step = (high - low) / bins
        counts = [0] * bins
        for value in values:
            index = min(bins - 1, max(0, int((value - low) / step)))
            counts[index] += 1
        labels = [f"{format_number(low + step * i, 1)}-{format_number(low + step * (i + 1), 1)}" for i in range(bins)]
    return svg_bar_chart(title, labels, [float(value) for value in counts], colors=["#64748b"] * len(counts), value_suffix=" runs")


def svg_seed_completion(report: dict[str, Any]) -> str:
    rows = list(report["results"])
    labels = [str(row["seed"])[-4:] for row in rows]
    values = []
    colors = []
    for row in rows:
        value = numeric(row.get("final_path_completion_fraction"))
        values.append(1.0 if value is None and row.get("success") else value or 0.0)
        colors.append("#15803d" if row.get("success") else "#b91c1c")
    return svg_bar_chart("Per-seed path completion", labels, values, colors=colors, max_value=1.0, value_suffix=" completion")


def svg_failure_taxonomy(kpis: dict[str, Any]) -> str:
    counts = kpis["reliability"]["failure_taxonomy"]
    labels = list(counts.keys())
    values = [float(counts[label]) for label in labels]
    return svg_bar_chart("Dominant failure modes", labels, values, colors=["#b91c1c"] * len(values), value_suffix=" runs")


def svg_goal_error_histogram(report: dict[str, Any]) -> str:
    return svg_histogram("Final goal error distribution", values_for(report["results"], "physical_goal_error_m"), suffix=" m")


def svg_coverage_scatter(report: dict[str, Any]) -> str:
    rows = [
        row for row in report["results"]
        if numeric(row.get("coverage_fraction")) is not None and numeric(row.get("physical_goal_error_m")) is not None
    ]
    if not rows:
        return "<section class='plot'><h3>Coverage vs final goal error</h3><p class='muted'>No data available.</p></section>"
    width, height = 760, 280
    margin_left, margin_top, margin_bottom = 58, 28, 44
    plot_w, plot_h = width - margin_left - 18, height - margin_top - margin_bottom
    errors = [numeric(row["physical_goal_error_m"]) or 0.0 for row in rows]
    max_error = max(errors) or 1.0
    parts = [
        "<section class='plot'><h3>Coverage vs final goal error</h3>",
        f"<svg viewBox='0 0 {width} {height}' role='img'><rect width='100%' height='100%' fill='white'/>",
        f"<line x1='{margin_left}' y1='{margin_top + plot_h}' x2='{width - 12}' y2='{margin_top + plot_h}' stroke='#d9dee7'/>",
        f"<line x1='{margin_left}' y1='{margin_top}' x2='{margin_left}' y2='{margin_top + plot_h}' stroke='#d9dee7'/>",
        f"<text x='{margin_left}' y='{height - 10}' font-size='11' fill='#6b7280'>map coverage</text>",
        f"<text x='8' y='{margin_top + 10}' font-size='11' fill='#6b7280'>goal error</text>",
    ]
    for row in rows:
        coverage = numeric(row["coverage_fraction"]) or 0.0
        error = numeric(row["physical_goal_error_m"]) or 0.0
        x = margin_left + plot_w * max(0.0, min(1.0, coverage))
        y = margin_top + plot_h - plot_h * max(0.0, min(1.0, error / max_error))
        color = "#15803d" if row.get("success") else "#b91c1c"
        parts.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='5' fill='{color}' opacity='0.88'><title>seed {row['seed']}: coverage {format_percent(coverage)}, error {format_number(error)} m</title></circle>")
    parts.append("</svg></section>")
    return "".join(parts)


def render_html_report(report: dict[str, Any], compare: dict[str, Any] | None = None) -> str:
    kpis = report.get("kpis") or aggregate_kpis(report, compare)
    report["kpis"] = kpis
    report["verdict"] = report.get("verdict") or verdict(kpis)
    decision = report["verdict"]
    success = kpis["success"]
    mission = kpis["mission"]
    safety = kpis["safety_motion"]
    localization = kpis["localization"]
    mapping = kpis["mapping"]
    data_quality = kpis["data_quality"]
    reliability = kpis["reliability"]
    dominant = reliability["dominant_failure_modes"]
    dominant_text = ", ".join(f"{name} ({count})" for name, count in dominant) if dominant else "none observed"
    compare_gap = success.get("seen_vs_heldout_gap")
    compare_sentence = (
        f" Seen-vs-held-out gap is {format_percent(compare_gap)}."
        if compare_gap is not None else
        " Seen-vs-held-out gap is not available because no comparison root was provided."
    )
    blocks = [
        metric_block("Success", "Held-out solve rate", [
            ("Reached goal", f"{success['goal_reached_count']}/{success['count']}", ""),
            ("Solve rate", success["solve_rate"], "%"),
            ("95% CI low", success["wilson_95_ci"]["low"], "%"),
            ("Seen-vs-held-out gap", compare_gap, "%"),
        ]),
        metric_block("Mission", "Efficiency and speed", [
            ("Path efficiency median", mission["path_efficiency_median"], "x"),
            ("Time-to-goal median", mission["time_to_goal_median_sim_s"], "sim-s"),
            ("Final goal error median", mission["final_goal_error_median_m"], "m"),
            ("Final goal error p95", mission["final_goal_error_p95_m"], "m"),
        ]),
        metric_block("Safety and motion", "Did it move cleanly?", [
            ("Wall-contact runs", safety["wall_contact_runs"], "runs"),
            ("Total wall contacts", safety["total_wall_contacts"], ""),
            ("Recovery events", safety["recovery_events_total"], ""),
            ("Clean-motion rate", safety["clean_motion_rate"], "%"),
        ]),
        metric_block("Localization", "Did it know where it was?", [
            ("ATE RMSE median", localization["ate_rmse_median_m"], "m"),
            ("ATE RMSE p95", localization["ate_rmse_p95_m"], "m"),
            ("Final odom error median", localization["final_error_median_m"], "m"),
            ("Drift scale median", localization["drift_scale_median"], "x"),
        ]),
        metric_block("Mapping", "Did it map enough?", [
            ("Coverage median", mapping["coverage_median"], "%"),
            ("Coverage p05", mapping["coverage_p05"], "%"),
            ("Known cells median", mapping["known_cells_median"], ""),
            ("Missing seeds", success["missing_count"], ""),
        ]),
        metric_block("Data quality and reliability", "Would it run unsupervised?", [
            ("Realtime factor median", data_quality["realtime_factor_median"], "x"),
            ("Schema completeness min", data_quality["schema_completeness_min"], "%"),
            ("MTBF", reliability["mtbf_m_per_failure"], "m/failure"),
            ("Crash-safe summaries", reliability["crash_safe_summary_rate"], "%"),
        ]),
    ]
    rows = []
    for row in report["results"]:
        rows.append(
            "<tr>"
            f"<td>{row['seed']}</td>"
            f"<td class=\"{'pass' if row['success'] else 'fail'}\">{'PASS' if row['success'] else 'FAIL'}</td>"
            f"<td>{escape(str(row.get('final_status')))}</td>"
            f"<td>{escape(str(row.get('failure_taxonomy')))}</td>"
            f"<td>{format_metric(row.get('physical_goal_error_m'), 'm')}</td>"
            f"<td>{format_metric(row.get('path_efficiency'), 'x')}</td>"
            f"<td>{format_metric(row.get('coverage_fraction'), '%')}</td>"
            f"<td>{format_metric(row.get('localization_rmse_m'), 'm')}</td>"
            f"<td>{escape(row.get('run_dir', ''))}</td>"
            "</tr>"
        )
    plots = [
        svg_seed_completion(report),
        svg_goal_error_histogram(report),
        svg_failure_taxonomy(kpis),
        svg_coverage_scatter(report),
    ]
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{escape(report['label'])} KPI Report</title>
<style>
body{{margin:0;background:#fff;color:#171717;font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;line-height:1.45}}
header{{padding:24px 28px;border-bottom:1px solid #d9dee7;display:flex;justify-content:space-between;gap:18px;align-items:flex-start}}
h1{{font-size:24px;margin:0 0 6px}}h2{{font-size:18px;margin:24px 0 10px}}h3{{font-size:15px;margin:0 0 8px}}p{{margin:0 0 12px}}main{{padding:20px 28px 34px}}.muted{{color:#6b7280}}.verdict{{border:1px solid #d9dee7;border-radius:8px;padding:14px 16px;max-width:420px}}.verdict b{{display:block;font-size:18px;margin-bottom:4px}}.good{{color:#15803d}}.warn{{color:#b45309}}.bad{{color:#b91c1c}}
.metrics{{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}}.metric-block{{min-width:0}}.eyebrow{{font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:#6b7280;margin:0 0 4px}}.metric-rows{{border-top:1px solid #d9dee7}}.metric-row{{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:10px;border-bottom:1px solid #d9dee7;padding:6px 0;min-height:30px}}.metric-row span{{font-size:13px}}.metric-row b{{font-size:13px;color:#4b5563;font-variant-numeric:tabular-nums;text-align:right}}
.writeup{{max-width:980px}}.plots{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}}.plot{{border-top:1px solid #d9dee7;padding-top:10px}}svg{{width:100%;height:auto}}table{{width:100%;border-collapse:collapse;font-size:12px}}th,td{{border-bottom:1px solid #d9dee7;padding:7px 6px;text-align:left;vertical-align:top}}th{{color:#6b7280;font-weight:650}}.pass{{color:#15803d;font-weight:700}}.fail{{color:#b91c1c;font-weight:700}}
@media(max-width:980px){{header{{display:block}}.metrics,.plots{{grid-template-columns:1fr}}}}
</style>
</head>
<body>
<header>
<div><h1>{escape(report['label'])} Navigation KPI Report</h1><p class="muted">Held-out seeds: {report['goal_reached_count']}/{report['count']} reached the generated maze goal. 95% Wilson CI: {format_percent(success['wilson_95_ci']['low'])} to {format_percent(success['wilson_95_ci']['high'])}.</p></div>
<aside class="verdict"><b class="{escape(decision['class'])}">{escape(decision['label'])}</b><p>{escape(decision['text'])}</p></aside>
</header>
<main>
<section class="metrics">{''.join(blocks)}</section>
<section class="writeup">
<h2>One-to-two-page analysis writeup</h2>
<p><b>Design.</b> The evaluated stack is a cold-start MuJoCo/ROS 2 navigation system: simulated Livox/D435i sensing feeds sensor-derived odometry, SLAM Toolbox mapping, m-explore/Nav2 goal selection, and a Unitree G1 locomotion policy. Ground truth is recorded only for scoring and dashboard comparison, not for navigation.</p>
<p><b>Tradeoffs.</b> The batch runner disables the live dashboard and prebuilds once to keep held-out runs comparable and low-overhead. The report favors held-out solve rate, physical goal error, contacts, localization RMSE, map coverage, schema completeness, and realtime factor over prettier online visuals. Some fine-grained live KPIs, such as inter-sensor skew and command jerk, require the per-run live stream or rosbag if they are not present in final summaries.</p>
<p><b>Dominant failure modes.</b> The main observed failure categories are {escape(dominant_text)}. Failures are classified from terminal status, wall contacts, and goal/timeout outcomes. The per-seed table below links each failure back to its run directory for inspection.</p>
<p><b>Verdict.</b> {escape(decision['text'])}{escape(compare_sentence)} This verdict should be revisited whenever the held-out seed set, cell size, duration, locomotion policy, or navigation parameters change.</p>
</section>
<section><h2>Plots</h2><div class="plots">{''.join(plots)}</div></section>
<section><h2>Per-seed results</h2><table><thead><tr><th>Seed</th><th>Result</th><th>Status</th><th>Failure mode</th><th>Goal error</th><th>Path efficiency</th><th>Coverage</th><th>ATE RMSE</th><th>Run dir</th></tr></thead><tbody>{''.join(rows)}</tbody></table></section>
</main>
</body>
</html>
"""


def format_percent(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1%}"


def print_report(report: dict[str, Any], compare: dict[str, Any] | None = None) -> None:
    label = report["label"]
    count = int(report["count"])
    successes = int(report["goal_reached_count"])
    rate = report["solve_rate"]
    interval = report["wilson_95_ci"]
    print(f"{label}_solve_rate: {successes}/{count} = {format_percent(rate)}")
    print(f"95% Wilson CI: [{format_percent(interval['low'])}, {format_percent(interval['high'])}]")
    expected = report.get("expected_count")
    if expected is not None:
        missing = report["missing_seeds"]
        print(f"expected seeds: {expected}; found summaries: {count}; missing: {len(missing)}")
        if missing:
            print("missing_seeds: " + " ".join(str(seed) for seed in missing))
    if compare is not None:
        compare_rate = compare["solve_rate"]
        heldout_rate = rate
        if compare_rate is not None and heldout_rate is not None:
            gap = compare_rate - heldout_rate
            print(
                f"{compare['label']} solve rate - {label} solve rate = "
                f"{format_percent(gap)} overfit gap"
            )
    print()
    for row in report["results"]:
        result = "PASS" if row["success"] else "FAIL"
        print(
            f"{row['seed']}: {result} status={row['final_status']} "
            f"goal_error_m={row['physical_goal_error_m']} {row['summary_path']}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("runs/heldout-20/navigate"))
    parser.add_argument("--label", default="held_out")
    parser.add_argument("--seeds", nargs="*", default=[])
    parser.add_argument("--default-heldout-seeds", action="store_true")
    parser.add_argument("--success-field", default="maze_goal_reached")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-html", type=Path)
    parser.add_argument("--compare-root", type=Path)
    parser.add_argument("--compare-label", default="seen")
    parser.add_argument("--compare-seeds", nargs="*", default=[])
    args = parser.parse_args()

    seeds = DEFAULT_HELDOUT_SEEDS if args.default_heldout_seeds else parse_seed_values(args.seeds)
    report = summarize(args.root, label=args.label, seeds=seeds, success_field=args.success_field)
    compare = None
    if args.compare_root:
        compare_seeds = parse_seed_values(args.compare_seeds)
        compare = summarize(
            args.compare_root,
            label=args.compare_label,
            seeds=compare_seeds,
            success_field=args.success_field,
        )
    write_report_files(report, args.output_json, args.output_csv, args.output_html, compare)
    print_report(report, compare)
    if args.output_html:
        print(f"html_report: {args.output_html}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
