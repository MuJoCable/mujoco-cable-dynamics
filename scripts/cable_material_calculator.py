#!/usr/bin/env python3
"""Convert effective cable material properties to the plugin stiffness."""

from __future__ import annotations

import argparse
import json
import math


def cable_material_properties(
    *,
    young_modulus_pa: float,
    diameter_m: float,
    reference_length_m: float,
    tension_n: float,
    poisson_ratio: float | None = None,
) -> dict[str, float | None]:
    if young_modulus_pa <= 0 or diameter_m <= 0 or reference_length_m <= 0:
        raise ValueError("Young's modulus, diameter, and reference length must be positive")
    if tension_n < 0:
        raise ValueError("tension must be nonnegative")
    if poisson_ratio is not None and not (-1.0 < poisson_ratio < 0.5):
        raise ValueError("isotropic Poisson ratio must lie between -1 and 0.5")

    area_m2 = math.pi * diameter_m**2 / 4.0
    axial_rigidity_n = young_modulus_pa * area_m2
    stiffness_n_per_m = axial_rigidity_n / reference_length_m
    axial_strain = tension_n / axial_rigidity_n
    extension_m = axial_strain * reference_length_m
    transverse_strain = (
        -poisson_ratio * axial_strain if poisson_ratio is not None else None
    )
    diameter_change_m = (
        transverse_strain * diameter_m if transverse_strain is not None else None
    )
    return {
        "area_m2": area_m2,
        "axial_rigidity_n": axial_rigidity_n,
        "plugin_stiffness_n_per_m": stiffness_n_per_m,
        "axial_strain": axial_strain,
        "extension_m": extension_m,
        "transverse_strain": transverse_strain,
        "diameter_change_m": diameter_change_m,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--young-modulus-mpa", type=float, required=True)
    parser.add_argument("--diameter-mm", type=float, required=True)
    parser.add_argument("--reference-length-m", type=float, required=True)
    parser.add_argument("--tension-n", type=float, default=0.0)
    parser.add_argument("--poisson-ratio", type=float)
    args = parser.parse_args()
    result = cable_material_properties(
        young_modulus_pa=args.young_modulus_mpa * 1e6,
        diameter_m=args.diameter_mm * 1e-3,
        reference_length_m=args.reference_length_m,
        tension_n=args.tension_n,
        poisson_ratio=args.poisson_ratio,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
