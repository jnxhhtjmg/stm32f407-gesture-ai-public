"""Convert the frozen INT8 LiteRT model into a Keil-friendly C byte array."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("output_dir", type=Path, help="Default directory for generated files")
    parser.add_argument("--header-dir", type=Path)
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--symbol", default="g_gesture_model_data")
    args = parser.parse_args()

    data = args.model.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    header_name = f"{args.symbol}.h"
    source_name = f"{args.symbol}.c"

    header = f"""#ifndef GESTURE_MODEL_DATA_H
#define GESTURE_MODEL_DATA_H

#include <stdint.h>

extern const unsigned char {args.symbol}[];
extern const unsigned int {args.symbol}_len;

#define GESTURE_MODEL_SHA256 \"{digest}\"
#define GESTURE_MODEL_BYTES {len(data)}u

#endif
"""

    lines = []
    for start in range(0, len(data), 12):
        chunk = data[start : start + 12]
        lines.append("    " + ", ".join(f"0x{byte:02x}" for byte in chunk) + ",")
    source = f"""#include \"{header_name}\"

#if defined(__CC_ARM)
__align(4)
#elif defined(__ARMCC_VERSION)
__attribute__((aligned(4)))
#endif
const unsigned char {args.symbol}[] = {{
{chr(10).join(lines)}
}};

const unsigned int {args.symbol}_len = (unsigned int)sizeof({args.symbol});
"""

    header_dir = args.header_dir or args.output_dir
    source_dir = args.source_dir or args.output_dir
    header_dir.mkdir(parents=True, exist_ok=True)
    source_dir.mkdir(parents=True, exist_ok=True)
    (header_dir / header_name).write_text(header, encoding="ascii")
    (source_dir / source_name).write_text(source, encoding="ascii")
    print(f"model_bytes={len(data)}")
    print(f"sha256={digest}")
    print(f"header={(header_dir / header_name).resolve()}")
    print(f"source={(source_dir / source_name).resolve()}")


if __name__ == "__main__":
    main()
