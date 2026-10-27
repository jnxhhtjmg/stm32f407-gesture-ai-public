"""Build leakage-safe IMU gesture windows from raw CSV captures.

Two window modes are supported:

* ``sliding`` - the original 1 s window / 0.5 s stride over the whole capture.
  Kept for reproducing the old baseline.
* ``onset``   - one window per requested offset after the detected motion
  onset.  This removes windows that only contain the return phase of a
  round-trip gesture, which otherwise carry the opposite class label.

The script stores **raw physical units** (g and deg/s).  Normalization is
deliberately left to the training script so that grouped cross-validation can
compute mean/std on each training fold only.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FEATURES = ("ax", "ay", "az", "gx", "gy", "gz")
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
ONSET_GYRO_DPS = 3.0
ONSET_ACCEL_G = 0.03
ONSET_CONSECUTIVE = 5


@dataclass(frozen=True)
class Capture:
    path: Path
    label: str
    domain: str
    values: np.ndarray  # (N, 6) physical units


def read_capture(path: Path, domain: str) -> Capture:
    values: list[list[float]] = []
    labels: set[str] = set()
    timestamps: list[int] = []

    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        required = {"timestamp", *FEATURES, "label"}
        missing = required.difference(header)
        if missing:
            raise ValueError(f"{path.name}: missing columns {sorted(missing)}")
        index = {name: header.index(name) for name in required}

        for row in reader:
            if len(row) < len(header):
                continue
            timestamps.append(int(row[index["timestamp"]]))
            values.append([float(row[index[name]]) for name in FEATURES])
            labels.add(row[index["label"]].strip())

    if not values:
        raise ValueError(f"{path.name}: empty capture")
    if len(labels) != 1:
        raise ValueError(f"{path.name}: expected one label, got {sorted(labels)}")
    if any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{path.name}: timestamps are not strictly increasing")

    array = np.asarray(values, dtype=np.float32)
    array[:, 0:3] /= ACCEL_LSB_PER_G
    array[:, 3:6] /= GYRO_LSB_PER_DPS
    return Capture(path=path, label=labels.pop(), domain=domain, values=array)


def load_captures(raw_dirs: list[Path]) -> list[Capture]:
    captures: list[Capture] = []
    for raw_dir in raw_dirs:
        found = [read_capture(path, raw_dir.name) for path in sorted(raw_dir.glob("*.csv"))]
        if not found:
            raise FileNotFoundError(f"No CSV captures found in {raw_dir}")
        captures.extend(found)
    return captures


def detect_onset(values: np.ndarray) -> int:
    """Return the first sample of sustained motion, or 0 if none is found."""
    baseline = values[: min(20, len(values)), 0:3].mean(axis=0)
    accel_dev = np.linalg.norm(values[:, 0:3] - baseline, axis=1)
    gyro_mag = np.linalg.norm(values[:, 3:6], axis=1)
    moving = (gyro_mag > ONSET_GYRO_DPS) | (accel_dev > ONSET_ACCEL_G)

    for start in range(0, len(moving) - ONSET_CONSECUTIVE + 1):
        if moving[start : start + ONSET_CONSECUTIVE].all():
            return start
    return 0


def extract_window(values: np.ndarray, start: int, size: int) -> np.ndarray:
    if start < 0:
        start = 0
    window = values[start : start + size]
    if len(window) == size:
        return window
    pad = np.repeat(window[-1:], size - len(window), axis=0)
    return np.vstack([window, pad])


def make_windows(
    capture: Capture,
    mode: str,
    offsets: list[int],
    size: int,
    stride: int,
    downsample: int,
) -> list[tuple[int, int, np.ndarray]]:
    """Return (onset, offset, window) triples."""
    onset = detect_onset(capture.values) if mode != "sliding" else 0
    windows: list[tuple[int, int, np.ndarray]] = []

    if mode == "sliding":
        for start in range(0, len(capture.values) - size + 1, stride):
            windows.append((onset, start, capture.values[start : start + size]))
        return windows

    if mode == "sequence":
        raw_size = size * downsample
        raw = extract_window(capture.values, onset, raw_size)
        if downsample > 1:
            raw = raw.reshape(size, downsample, len(FEATURES)).mean(axis=1)
        return [(onset, 0, raw)]

    for offset in offsets:
        start = onset + offset
        if start >= len(capture.values):
            continue
        windows.append((onset, offset, extract_window(capture.values, start, size)))

    if not windows:
        windows.append((onset, 0, extract_window(capture.values, 0, size)))
    return windows


def stratified_group_split(captures: list[Capture], seed: int) -> dict[str, str]:
    """70/15/15 capture-level split, stratified by label."""
    grouped: dict[tuple[str, str], list[Capture]] = {}
    for capture in captures:
        grouped.setdefault((capture.domain, capture.label), []).append(capture)

    rng = random.Random(seed)
    split: dict[str, str] = {}
    for key in sorted(grouped):
        items = list(grouped[key])
        rng.shuffle(items)
        count = len(items)
        if count <= 12:
            train_end = max(1, round(count * 0.60))
            validation_end = max(train_end + 1, round(count * 0.80))
        else:
            train_end = max(1, round(count * 0.70))
            validation_end = max(train_end + 1, round(count * 0.85))
        validation_end = min(validation_end, count - 1)
        for index, capture in enumerate(items):
            if index < train_end:
                split[capture.path.name] = "train"
            elif index < validation_end:
                split[capture.path.name] = "validation"
            else:
                split[capture.path.name] = "test"
    return split


def phase_consistency(
    captures: list[Capture], windows: dict[str, np.ndarray]
) -> dict[str, object]:
    """How often the first half of a window points in the capture's direction.

    This is a diagnostic, not a filter: a low value means the capture label and
    the window content disagree (typically because the window landed on the
    return phase of a round trip).
    """
    stats: dict[str, object] = {}
    for first, second in (
        ("move_left", "move_right"),
        ("tilt_left", "tilt_right"),
    ):
        first_values = windows.get(first, np.empty((0, 100, 6)))
        second_values = windows.get(second, np.empty((0, 100, 6)))
        if len(first_values) == 0 or len(second_values) == 0:
            continue

        # Pick the axis that separates the two classes most strongly instead of
        # hard-coding ax/gz: a "move" is mostly a gyro pattern, a "tilt" is
        # mostly gz, and the data-driven choice stays valid if the mount changes.
        first_mean = first_values[:, :50, :].mean(axis=(0, 1))
        second_mean = second_values[:, :50, :].mean(axis=(0, 1))
        axis = int(np.argmax(np.abs(first_mean - second_mean)))
        expected = 1.0 if first_mean[axis] >= second_mean[axis] else -1.0
        reference = (first_mean[axis] + second_mean[axis]) / 2.0
        values = first_values[:, :50, axis].mean(axis=1)
        consistent = int(np.sum(np.sign(values - reference) == expected))
        stats[first] = {
            "axis": FEATURES[axis],
            "windows": int(len(values)),
            "consistent": consistent,
            "fraction": round(consistent / len(values), 4),
        }
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--raw-dir",
        type=Path,
        nargs="+",
        default=[Path("data/raw"), Path("data/raw_f407")],
    )
    parser.add_argument("--output-dir", type=Path, default=Path("data/processed_onset"))
    parser.add_argument(
        "--mode", choices=("onset", "sliding", "sequence"), default="onset"
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=2,
        help="Sequence mode only: raw 100 Hz samples per output sample",
    )
    parser.add_argument(
        "--offsets",
        type=int,
        nargs="+",
        default=(25, 50, 75),
        help="Window starts after the detected onset (onset mode only)",
    )
    parser.add_argument("--window-size", type=int, default=100)
    parser.add_argument("--stride", type=int, default=50, help="Sliding mode stride")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    captures = load_captures(args.raw_dir)
    labels = sorted({capture.label for capture in captures})
    label_index = {label: index for index, label in enumerate(labels)}
    split = stratified_group_split(captures, args.seed)

    features: list[np.ndarray] = []
    targets: list[int] = []
    sources: list[str] = []
    domains: list[str] = []
    onsets: list[int] = []
    offsets_used: list[int] = []
    per_label_list: dict[str, list[np.ndarray]] = {}

    for capture in captures:
        for onset, offset, window in make_windows(
            capture,
            args.mode,
            list(args.offsets),
            args.window_size,
            args.stride,
            args.downsample,
        ):
            features.append(window)
            targets.append(label_index[capture.label])
            sources.append(capture.path.name)
            domains.append(capture.domain)
            onsets.append(onset)
            offsets_used.append(offset)
            per_label_list.setdefault(capture.label, []).append(window)

    per_label = {
        label: np.stack(items) for label, items in per_label_list.items()
    }

    x = np.stack(features).astype(np.float32)
    y = np.asarray(targets, dtype=np.int64)
    source_array = np.asarray(sources)
    split_array = np.asarray([split[name] for name in sources])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output_dir / "dataset.npz",
        X=x,
        y=y,
        source=source_array,
        domain=np.asarray(domains),
        split=split_array,
        onset=np.asarray(onsets, dtype=np.int32),
        offset=np.asarray(offsets_used, dtype=np.int32),
    )

    metadata = {
        "features": list(FEATURES),
        "label_names": labels,
        "window_mode": args.mode,
        "window_size": args.window_size,
        "offsets": list(args.offsets) if args.mode == "onset" else None,
        "stride": args.stride if args.mode == "sliding" else None,
        "raw_sampling_rate_hz": 100,
        "downsample": args.downsample if args.mode == "sequence" else 1,
        "sampling_rate_hz": 100.0 / (args.downsample if args.mode == "sequence" else 1),
        "window_seconds": (
            args.window_size * args.downsample / 100.0
            if args.mode == "sequence"
            else args.window_size / 100.0
        ),
        "units": {
            "acceleration": "g",
            "angular_velocity": "degrees_per_second",
        },
        "sensor_scale": {
            "acceleration_lsb_per_g": ACCEL_LSB_PER_G,
            "angular_velocity_lsb_per_degree_per_second": GYRO_LSB_PER_DPS,
        },
        "onset": {
            "gyro_dps": ONSET_GYRO_DPS,
            "accel_g": ONSET_ACCEL_G,
            "consecutive_samples": ONSET_CONSECUTIVE,
        },
        "seed": args.seed,
        "domains": sorted(set(domains)),
        "domain_counts": {
            domain: sum(1 for capture in captures if capture.domain == domain)
            for domain in sorted({capture.domain for capture in captures})
        },
        "capture_counts": {
            name: sum(1 for value in split.values() if value == name)
            for name in ("train", "validation", "test")
        },
        "window_counts": {
            name: int(np.sum(split_array == name))
            for name in ("train", "validation", "test")
        },
        "class_counts": {
            label: int(np.sum(y == label_index[label])) for label in labels
        },
        "phase_consistency": phase_consistency(captures, per_label),
    }
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    (args.output_dir / "split.json").write_text(
        json.dumps(split, indent=2, sort_keys=True), encoding="utf-8"
    )

    print(f"Captures: {len(captures)}")
    print(f"Labels: {', '.join(labels)}")
    print(f"Mode: {args.mode}")
    print(f"Windows: {len(x)}")
    print(f"Capture split: {metadata['capture_counts']}")
    print(f"Window split: {metadata['window_counts']}")
    print(f"Saved: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
