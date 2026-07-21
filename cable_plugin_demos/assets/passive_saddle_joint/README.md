# Passive Saddle-Joint Assets

`test_1_visual.stl` and `test_2_visual.stl` are user-provided source meshes copied from the local `evoArm` model. The first source contains micrometer-scale intersections between non-adjacent triangles, so strict route validation rejects it.

`test_1_routed.stl` and `test_2_routed.stl` are watertight voxel-remeshed and quadric-decimated versions of those same joint surfaces. Demo 16 and Demo 20 use each routed mesh for both rendering and `mesh_route_mode="taut_obstacle"` cable routing, so there is no separate route-proxy shape.

Regenerate the routed meshes with Blender, then regenerate the contact assets and manifest:

```bash
blender --background --factory-startup \
  --python scripts/remesh_saddle_route_assets.py -- \
  --asset-dir cable_plugin_demos/assets/passive_saddle_joint

conda run -n rope_plugin python scripts/prepare_passive_saddle_assets.py
```

The generated files have separate physical roles:

- `*_contact_patch.obj`: deterministic low-face nonconvex saddle patches used by rigid `flexcomp` contact.
- `test_*_visual.stl`: retained user source geometry.
- `test_*_routed.stl`: closed, manifold, non-self-intersecting visual and routing geometry.
- `asset_manifest.json`: source/routed topology checks, remesh provenance, neutral-pose scan, and contact-patch fit diagnostics.

The routed meshes are closed, manifold, consistently oriented, nonconvex, and accepted by the plugin's BVH self-intersection validator. Demo 16 and Demo 20 list their rendered mesh geoms directly in `wrap_geoms`; the low-face contact patches remain separate because high-resolution rigid mesh contact is not used here.
