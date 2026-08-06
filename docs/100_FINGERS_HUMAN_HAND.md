# 100_fingers Human Hand in MuJoCo

## Scope

Demo 26 reconstructs the human preset from the 100_fingers resource as a
MuJoCo articulated hand driven by MuJoCable. It uses the source project's
actual palm and phalanx surfaces, published dimensions, four-DOF digit
structure, and five-tendon functional topology. It is a dynamics-oriented
reconstruction, not a drop-in simulation of the source living hinges or its
external mixer box.

Each digit has 4 DOF: MCP/CMC abduction-adduction, MCP/CMC flexion-extension,
PIP flexion-extension, and DIP flexion-extension.

Each digit has 5 independently controllable unilateral cables:

| Cable | Color | Route and role |
|---|---|---|
| Extensor | Blue | Dorsal route over MCP, PIP, and DIP |
| Abductor | Cyan | One side of the MCP/CMC lateral pulley |
| Adductor | Magenta | Opposite side of the MCP/CMC lateral pulley |
| Intermediate flexor | Orange | Palmar route over MCP and PIP, anchored on the intermediate phalanx |
| Distal flexor | Red | Palmar route over MCP, PIP, and DIP, anchored on the distal phalanx |

The force-disabled seed tendons only encode endpoint/hint order. MuJoCable
computes the visible runtime envelopes, cable lengths, slack state, tensions,
and generalized forces. The wrap cylinders are invisible analytic surfaces
placed at the inferred joint centers; they are not replacement visual bones.
Role-3 sites reproduce the source design's physical tendon guides along each
phalanx.

## Run

Build the plugin, then run the coordinated close-hold-open cycle:

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --duration 120
```

On Linux, replace `mjpython` with `python3`. For direct manual control:

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --manual --duration 120
```

### Index-only routing debug model

Demo 27 retains the complete hand mesh and all 20 joints but removes the
non-index cable instances, seed tendons, actuators, plugin sensors, and route
debug sites. The standard Control panel therefore exposes only five index
cables:

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml \
  --camera index_route_debug \
  --manual --show-route-debug --duration 120
```

Debug colors are red for physical endpoints (role 1), yellow for initialization
hints (role 2), cyan for persistent physical guides (role 3), and translucent
green for analytic wrap cylinders. The viewer prints every role site, owning
body, and local `pos` to the terminal for direct MJCF editing.

Without `--manual`, the same command runs a repeatable index close-hold-open
cycle. Positive control is target cable contraction in metres.

### Demo 28: actual-mesh threading

Demo 28 removes the analytic green cylinders. The proximal, intermediate, and
distal index STLs are topology-repaired into closed, manifold,
self-intersection-free route meshes. Each repaired mesh is both the visible
phalanx and the named `wrap_geom`, so the rendered finger surface is the
surface used by the cable solver.

The source STL components contain a few nonmanifold edges and intersecting
patches and therefore cannot be passed directly to `guided_surface`. The
developer asset step uses 0.5 mm voxelization, solid fill, inward marching
cubes, and closed-volume-preserving decimation. It preserves the overall form
and resolved pulley/groove geometry, but sub-voxel details are approximate.

Inspect one route at a time:

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/28_cpp_plugin_100_fingers_index_mesh_threading.xml \
  --camera index_route_debug \
  --manual --freeze \
  --debug-cable index_flexor_distal \
  --route-json index_flexor_distal_sites.json \
  --duration 120
```

`--debug-cable` fades the other seed routes, `--freeze` retains the neutral
pose, and `--route-json` exports site roles, owning bodies, and local
coordinates. Restart the viewer after editing the MJCF.

Demo 28 also adds two passive cable ligaments at each MCP, PIP, and DIP joint.
They have no actuator and use `home_length="auto_initial"`, 180 N/m stiffness,
and 0.15 mm pretension offset. The hinges remain, so these cables provide
additional compliant restoring forces; they do not yet replace the joint
kinematics. A hinge-free model requires free rigid bodies, contact surfaces,
and a separate stability validation.

## Threading Through a Reserved Mesh Hole

This is possible under explicit geometric and topological conditions:

1. The route mesh must contain the hole entrance, exit, and tunnel wall. A
   solid with a through-hole can still be a closed two-manifold; the lumen is
   free space outside the solid.
2. The plugin does not automatically discover which hole to use. Role-2 hints
   must select the intended homotopy branch. The same mesh may be repeated in
   `wrap_geoms` for entry and exit contact episodes.
3. The robust engineering approximation is a role-3 guide at each mouth. A
   guide is a force-bearing ideal eyelet, not distributed triangle contact.
4. To model tunnel-wall contact, preserve the wall in the route mesh and use a
   `guided_surface` corridor. High-genus through-hole routing still needs a
   dedicated acceptance test in this project.
5. A site placed inside a solid without a geometric hole does not create a
   hole. It causes penetration, `route_status=2`, and disables cable force for
   that step.

Start with two mouth guides and a collision-free span through the real lumen,
then add tunnel-wall surface contact if it is mechanically important. A
MuJoCo `site` has no collision geometry by itself.

## Position and Route Tuning

The preparation script estimates each joint center from the nearest interface
between adjacent source-mesh components. Tune one route at a time in this
order:

1. In Demo 27, align each `index_*_wrap` position, size, and orientation with the visible
   STL joint arc. The current cylinders are dimension-based estimates.
   Demo 28 has no cylinders; move role-2 hints to select the desired groove or
   side on the actual mesh.
2. Move a role-1 `*_start` or `*_end` site to change the physical attachment;
   keep it outside the rigid mesh.
3. Move a role-2 `*_hint` around the matching wrap cylinder to choose the
   initialized routing side. The hint is not a runtime force point.
4. Keep a role-3 `*_guide` only where the real hand has an eyelet or printed
   guide. It remains a runtime force point and therefore creates a real bend.
5. Keep the number and order of role-2 hints equal to `wrap_geoms`.
6. Keep the seed tendon width near zero. A thick seed is only MuJoCo's straight
   site-to-site polyline; plugin visualization is the true route.
7. Tune `stiffness`, `damping`, `slack`, and `pretension_offset` after geometry.

For example, the distal flexor topology is:

```text
start -> MCP hint -> proximal guide -> PIP hint
      -> intermediate guide -> DIP hint -> end
```

The three hints correspond, in order, to `index_mcp_wrap`, `index_pip_wrap`,
and `index_dip_wrap`. The two guides are persistent nodes. Moving a guide can
therefore change both appearance and generalized force, while moving a hint
only changes the initialized envelope branch.

The preparation script stores inferred geometric values in
`asset_manifest.json` in millimetres. To regenerate the index-only model after
changing Demo 26:

```bash
python3 scripts/create_100_fingers_index_debug_model.py
```

This overwrites Demo 27; make manual route changes in Demo 26 first or preserve
a separate working copy.

The source build guide gives extension-spring stiffnesses of approximately
0.06-1 N/mm (60-1000 N/m). Demo 26 uses 350-700 N/m, within that system-level
range. These values represent tendon-plus-series-spring compliance, not a
Poisson-ratio material model for bare nylon line.

## Current Limitations

- The living ligaments are represented by explicit joints, not deformable
  printed bridges.
- The source external nonlinear tendon mixer and spring racks are not yet
  modeled.
- Joint wrap guides are analytic cylinders inferred from the published joint
  diameters; cable contact is not yet solved directly on the high-resolution
  phalanx STL grooves.
- Self-collision and object-grasp contact are disabled in this first model so
  cable routing and actuation can be validated independently.
- Twenty-five simultaneous surface routes are substantially more expensive
  than the single-finger demos.

## Attribution

The hand mesh derives from the 100_fingers project and is redistributed under
CC BY 4.0. See `THIRD_PARTY_NOTICES.md` and the asset-directory README. The
OpenSCAD implementation remains GPLv3 in its source repository and is not
included here.

The source design is described by Gilday et al., "Embodied manipulation with
past and future morphologies through an open parametric hand design," *Science
Robotics*, vol. 10, no. 102, 2025, doi: 10.1126/scirobotics.ads6437.
