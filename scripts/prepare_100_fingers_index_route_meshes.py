#!/usr/bin/env python3
"""Rebuild the index phalanges as closed meshes for cable surface routing.

The published STL components contain a few nonmanifold edges and intersecting
surface patches. They are suitable for rendering, but not for a signed
inside/outside obstacle query. This developer-only asset step voxelizes the
actual STL surface, fills the solid, extracts a closed marching-cubes surface,
and applies conservative quadric decimation.

Runtime use of the generated OBJ files has no Python package dependency.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET_DIR = ROOT / "cable_plugin_demos/assets/100_fingers_human"
SPECS = (
    ("human_index_proximal.stl", "human_index_proximal_route.obj", 3000),
    ("human_index_intermediate.stl", "human_index_intermediate_route.obj", 2200),
    ("human_index_distal.stl", "human_index_distal_route.obj", 900),
)


def _dependencies():
    try:
        import numpy as np
        import trimesh
        from scipy import ndimage
        from skimage import measure
    except ImportError as error:
        raise RuntimeError(
            "Asset generation requires trimesh, scikit-image, and "
            "fast-simplification. Install them in a developer environment; "
            "they are not runtime dependencies."
        ) from error
    return np, trimesh, ndimage, measure


def _shrunken_marching_cubes(
    voxel,
    pitch_mm: float,
    np,
    trimesh,
    ndimage,
    measure,
):
    """Extract the isosurface half a voxel inside the filled occupancy."""
    occupancy = voxel.matrix.astype(bool)
    signed_distance = (
        ndimage.distance_transform_edt(occupancy)
        - ndimage.distance_transform_edt(~occupancy)
    )
    padded = np.pad(
        signed_distance,
        pad_width=1,
        mode="constant",
        constant_values=-2.0,
    )
    vertices, faces, normals, _ = measure.marching_cubes(
        padded,
        level=0.5,
        spacing=(pitch_mm, pitch_mm, pitch_mm),
    )
    vertices -= pitch_mm
    vertices += voxel.transform[:3, 3]
    rebuilt = trimesh.Trimesh(
        vertices=vertices,
        faces=faces,
        vertex_normals=normals,
        process=False,
    )
    rebuilt.fix_normals(multibody=True)
    return rebuilt


def _closed_candidate(mesh, desired_faces: int):
    original_faces = len(mesh.faces)
    targets = (
        desired_faces,
        int(desired_faces * 1.25),
        int(desired_faces * 1.5),
        int(desired_faces * 2.0),
        original_faces,
    )
    candidates = []
    for target in targets:
        if target >= original_faces:
            candidate = mesh.copy()
        else:
            candidate = mesh.simplify_quadric_decimation(
                face_count=target, aggression=0
            )
        candidate.remove_unreferenced_vertices()
        if (
            candidate.is_watertight
            and candidate.is_winding_consistent
            and candidate.is_volume
        ):
            candidates.append(candidate)
    if not candidates:
        raise RuntimeError("decimation did not preserve a closed oriented volume")
    return min(candidates, key=lambda item: len(item.faces))


def prepare(asset_dir: Path, pitch_mm: float) -> dict[str, object]:
    np, trimesh, ndimage, measure = _dependencies()
    report: dict[str, object] = {
        "generator": "scripts/prepare_100_fingers_index_route_meshes.py",
        "source_units": "mm",
        "output_units": "m",
        "voxel_pitch_mm": pitch_mm,
        "method": (
            "surface voxelization, parity solid fill, marching-cubes extraction, "
            "and closed-volume-preserving quadric decimation"
        ),
        "assets": {},
    }
    for source_name, output_name, target_faces in SPECS:
        source = trimesh.load_mesh(
            asset_dir / source_name, process=False, maintain_order=True
        )
        if not isinstance(source, trimesh.Trimesh):
            raise RuntimeError(f"{source_name} did not load as one triangle mesh")
        voxel = source.voxelized(pitch_mm).fill()
        rebuilt = _shrunken_marching_cubes(
            voxel, pitch_mm, np, trimesh, ndimage, measure
        )
        rebuilt = _closed_candidate(rebuilt, target_faces)
        rebuilt.apply_scale(0.001)
        rebuilt.remove_unreferenced_vertices()
        output = asset_dir / output_name
        output.write_bytes(trimesh.exchange.obj.export_obj(rebuilt).encode("ascii"))

        verified = trimesh.load_mesh(output, process=False, maintain_order=True)
        if not (
            verified.is_watertight
            and verified.is_winding_consistent
            and verified.is_volume
        ):
            raise RuntimeError(f"generated route mesh is invalid: {output_name}")
        report["assets"][source_name] = {
            "output": output_name,
            "source_faces": int(len(source.faces)),
            "route_vertices": int(len(verified.vertices)),
            "route_faces": int(len(verified.faces)),
            "watertight": bool(verified.is_watertight),
            "winding_consistent": bool(verified.is_winding_consistent),
            "positive_volume": bool(verified.is_volume),
            "source_bounds_mm": np.asarray(source.bounds).tolist(),
            "route_bounds_m": np.asarray(verified.bounds).tolist(),
        }

    manifest = asset_dir / "index_route_mesh_manifest.json"
    manifest.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument(
        "--pitch-mm",
        type=float,
        default=0.5,
        help="Voxel pitch used to repair the source surface, in millimetres.",
    )
    args = parser.parse_args()
    if args.pitch_mm <= 0:
        raise ValueError("--pitch-mm must be positive")
    print(
        json.dumps(
            prepare(args.asset_dir.expanduser().resolve(), args.pitch_mm),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
