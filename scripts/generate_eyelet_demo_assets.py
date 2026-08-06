#!/usr/bin/env python3
"""Generate deterministic low-poly assets for the eyelet demos."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "cable_plugin_demos/assets/eyelet/washer.obj"


def generate_washer(
    output: Path,
    *,
    segments: int = 48,
    outer_radius: float = 0.055,
    inner_radius: float = 0.025,
    half_thickness: float = 0.008,
) -> None:
    if segments < 8:
        raise ValueError("segments must be at least 8")
    if not 0 < inner_radius < outer_radius:
        raise ValueError("radii must satisfy 0 < inner < outer")
    if half_thickness <= 0:
        raise ValueError("half_thickness must be positive")

    vertices: list[tuple[float, float, float]] = []
    for radius, height in (
        (outer_radius, -half_thickness),
        (outer_radius, half_thickness),
        (inner_radius, -half_thickness),
        (inner_radius, half_thickness),
    ):
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            vertices.append(
                (radius * math.cos(angle), radius * math.sin(angle), height)
            )

    outer_bottom = 0
    outer_top = segments
    inner_bottom = 2 * segments
    inner_top = 3 * segments
    faces: list[tuple[int, int, int]] = []
    for index in range(segments):
        next_index = (index + 1) % segments
        ob_i, ob_j = outer_bottom + index, outer_bottom + next_index
        ot_i, ot_j = outer_top + index, outer_top + next_index
        ib_i, ib_j = inner_bottom + index, inner_bottom + next_index
        it_i, it_j = inner_top + index, inner_top + next_index

        faces.extend(
            (
                (ob_i, ob_j, ot_j),
                (ob_i, ot_j, ot_i),
                (ib_i, it_j, ib_j),
                (ib_i, it_i, it_j),
                (ot_i, ot_j, it_j),
                (ot_i, it_j, it_i),
                (ob_i, ib_j, ob_j),
                (ob_i, ib_i, ib_j),
            )
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# Closed annular washer for rigid-flex eyelet demos.\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.10g} {y:.10g} {z:.10g}\n")
        for a, b, c in faces:
            stream.write(f"f {a + 1} {b + 1} {c + 1}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--segments", type=int, default=48)
    args = parser.parse_args()
    generate_washer(args.output.resolve(), segments=args.segments)
    print(f"Wrote {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
