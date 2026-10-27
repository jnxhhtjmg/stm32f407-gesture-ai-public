"""Inspect capture timing, ranges, and labels before model training."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


FEATURES = ("ax", "ay", "az", "gx", "gy", "gz")


def inspect_file(path: Path) -> dict:
    frame = pd.read_csv(path)
    timestamps = frame["timestamp"].to_numpy(dtype=np.int64)
    intervals = np.diff(timestamps)
    label_values = frame["label"].dropna().astype(str).unique().tolist()
    return {
        "file": path.name,
        "label": label_values[0] if len(label_values) == 1 else label_values,
        "samples": int(len(frame)),
        "duration_ms": int(timestamps[-1] - timestamps[0]),
        "interval_ms": {
            "min": int(intervals.min()) if len(intervals) else None,
            "median": float(np.median(intervals)) if len(intervals) else None,
            "max": int(intervals.max()) if len(intervals) else None,
            "over_15ms": int((intervals > 15).sum()) if len(intervals) else 0,
        },
        "ranges": {
            feature: {
                "min": int(frame[feature].min()),
                "max": int(frame[feature].max()),
                "std": round(float(frame[feature].std()), 3),
            }
            for feature in FEATURES
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, default=Path("data/raw"))
    parser.add_argument(
        "--output", type=Path, default=Path("data/quality_report.json")
    )
    args = parser.parse_args()

    reports = [inspect_file(path) for path in sorted(args.raw_dir.glob("*.csv"))]
    if not reports:
        raise FileNotFoundError(f"No CSV files found in {args.raw_dir}")

    summary = {
        "file_count": len(reports),
        "labels": {
            label: sum(report["label"] == label for report in reports)
            for label in sorted({report["label"] for report in reports})
        },
        "files_with_interval_over_15ms": sum(
            report["interval_ms"]["over_15ms"] > 0 for report in reports
        ),
        "reports": reports,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Files: {summary['file_count']}")
    print(f"Labels: {summary['labels']}")
    print(
        "Files with an interval over 15 ms: "
        f"{summary['files_with_interval_over_15ms']}"
    )
    print(f"Saved: {args.output.resolve()}")


if __name__ == "__main__":
    main()
