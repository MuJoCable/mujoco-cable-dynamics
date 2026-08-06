# 100_fingers Human-Hand Assets

These files are deterministic derivatives of
`STLs/hands/human_hand_v3.1_nolig.stl` from the
[100_fingers parametric hand](https://github.com/kg398/100_fingers).
The source project identifies its 3D assets as licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

The source STL contains 16 disconnected components. The preparation script
extracts the largest component as the palm/metacarpal frame and classifies the
remaining 15 components as three phalanges for each of four fingers and one
thumb. Moving meshes are translated and rotated into joint-local frames; no
surface shape is synthesized or smoothed.

The three `human_index_*_route.obj` files are separate topology-repaired
derivatives for Demo 28. The source index components contain a small number of
nonmanifold edges and intersecting patches, which are invalid for signed
surface-obstacle queries. The route assets use 0.5 mm voxelization, solid fill,
an inward marching-cubes isosurface, and conservative decimation. Demo 28 uses
the same repaired surfaces for rendering and cable routing.

Regenerate the assets and Demo 26 with:

```bash
python scripts/prepare_100_fingers_human_hand.py \
  --source-stl <path-to-100_fingers-main>/STLs/hands/human_hand_v3.1_nolig.stl
```

`asset_manifest.json` records the source hash, inferred pivots, component gaps,
and local coordinate bases. It intentionally contains no local absolute path.

Regenerate the index route meshes in a developer environment with:

```bash
python -m pip install trimesh scikit-image fast-simplification
python scripts/prepare_100_fingers_index_route_meshes.py
python scripts/create_100_fingers_index_mesh_route_model.py
```

These Python packages are asset-generation dependencies only. Runtime loading
of the generated MJCF and OBJ files requires only MuJoCo and the cable plugin.
`index_route_mesh_manifest.json` records the repair resolution and topology
checks.
