"""Collect labeled F407 captures through the firmware's CSV capture mode.

The board toggles capture with KEY0 and prints `capture,start` / `capture,stop`
followed by 100 Hz CSV rows.  This host script adds the label column and saves
one file per press-to-press capture.

Example:
    python collect_f407.py --label lift --count 10 --port COM6
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import serial


def has_motion(rows: list[list[str]], min_gyro: float, min_accel: float) -> bool:
    if not rows:
        return False
    values = [[float(value) for value in row] for row in rows]
    gyro_max = max(
        math.sqrt(row[4] ** 2 + row[5] ** 2 + row[6] ** 2) / 131.0
        for row in values
    )
    count = min(20, len(values))
    baseline = [sum(row[i] for row in values[:count]) / count for i in (1, 2, 3)]
    accel_max = max(
        math.sqrt(
            (row[1] - baseline[0]) ** 2
            + (row[2] - baseline[1]) ** 2
            + (row[3] - baseline[2]) ** 2
        )
        / 16384.0
        for row in values
    )
    return gyro_max >= min_gyro or accel_max >= min_accel


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output-dir", type=Path, default=Path("data/raw_f407"))
    parser.add_argument("--min-samples", type=int, default=150)
    parser.add_argument("--max-samples", type=int, default=600)
    parser.add_argument("--skip", type=int, default=30, help="Samples dropped at capture start")
    parser.add_argument("--min-gyro", type=float, default=8.0, help="deg/s")
    parser.add_argument("--min-accel", type=float, default=0.03, help="g")
    parser.add_argument(
        "--allow-still", action="store_true", help="Accept captures without motion (neutral)"
    )
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    port = None
    rows: list[list[str]] = []
    saved = 0
    started = time.time()

    print(
        f"Collecting {args.count} x {args.label} from {args.port}.\n"
        "Press KEY0 to start a capture, perform the gesture, press KEY0 again."
    )

    while saved < args.count and (time.time() - started) < args.timeout:
        if port is None:
            try:
                port = serial.Serial(args.port, args.baud, timeout=0.5)
                port.reset_input_buffer()
            except Exception:
                time.sleep(0.5)
                continue

        try:
            line = port.readline()
        except Exception:
            try:
                port.close()
            except Exception:
                pass
            port = None
            time.sleep(0.5)
            continue

        if not line:
            continue
        text = line.decode("utf-8", "replace").strip()

        if text == "capture,start":
            rows = []
        elif text == "capture,stop":
            usable = rows[args.skip :]
            motion = has_motion(usable, args.min_gyro, args.min_accel)
            if (
                args.min_samples <= len(usable) <= args.max_samples
                and (motion or args.allow_still)
            ):
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                path = args.output_dir / f"f407_{args.label}_{stamp}.csv"
                with path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(
                        ["timestamp", "ax", "ay", "az", "gx", "gy", "gz", "label"]
                    )
                    for row in usable:
                        writer.writerow(row + [args.label])
                saved += 1
                print(f"saved {saved}/{args.count} ({len(usable)} samples)")
            else:
                print(
                    f"discarded: samples={len(usable)} motion={motion}"
                )
            rows = []
        elif text.count(",") == 6 and text[0].isdigit():
            rows.append(text.split(","))

    if handle is not None:
        handle.close()
    if port is not None:
        port.close()
    print(f"done: {saved}/{args.count} captures in {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
