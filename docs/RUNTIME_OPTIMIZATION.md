# Runtime Optimization and Control Update

This source update is based on public commit
`fca5aa5fcbfac362653c8b0888c0063e24a713a4`. It does not change the
massless-cable assumption or make general mesh routing a stateless solver.

## Route and Force Computation

- A solved wrap can reuse its local contacts when both neighboring hard sites
  and the obstacle belong to one rigid body. Reuse requires unchanged local
  inputs and a repeated, stable contact solution, not just an acceptable
  first initialization. World contacts and velocities are still updated.
- Internal self-visibility queries are memoized at identical local coordinates
  only. There is no coordinate quantization. Each geom has at most 2048 memo
  entries; inter-body spans and the final all-obstacle checks are not skipped.
- A mesh listed repeatedly in `wrap_geoms` retains its multiple route visits,
  but the final free-span obstacle check visits each distinct mesh only once.
- Repeated plugin data assignment preserves already-owned immutable mesh
  arrays for the same model/mesh. Mutable route state is still copied; separate
  data instances do not share writable corridors.
- Contact forces on each body are accumulated into one wrench before a
  single `mj_applyFT` call:

  ```text
  F_b   = sum_i F_i
  tau_b = sum_i (p_i - x_b) cross F_i
  qfrc_b = J_linear(x_b)^T F_b + J_angular(b)^T tau_b
  ```

  Here `p_i` is a contact or anchor position on body `b`, and `x_b` is that
  body's frame origin. The map is the same as applying the point forces
  separately, up to floating-point accumulation order. Segment-wise friction
  propagation happens before this accumulation and is retained.

No obstacle mesh is simplified and no contact or penetration check is disabled
by these optimizations. Geometry is assumed immutable after model compilation;
reload the model after editing mesh assets.

## Additional Optional Controls

| Configuration | Default | Meaning |
|---|---|---|
| `control_source_actuator` | unset | Read contraction commands from a named actuator, useful for linked cable branches. |
| `control_scale` | `1` | Scale shared/direct contraction commands, including target spool-angle commands. It is not a mechanical gearbox or a work-conserving differential. |
| `cylinder_winding_direction` | `auto` | Select `auto`, `positive`, or `negative` angular winding. |
| `cylinder_extra_turns` | `0` | Add a nonnegative integer number of turns in the selected cylinder branch. |
| `cylinder_min_wrap_angle` | `0` | Reject initial branches below this angle in radians; range `[0, 2*pi)`. |

Surface cables with plugin actuators can now use `capstan_direction=velocity`.
The estimated material velocity includes the contraction rate, with positive
contraction directed into the first seed endpoint. The rate is updated in the
plugin advance callback; a control discontinuity is not an instantaneous
material-velocity measurement. This remains a regularized sliding-friction
model, not exact static-stick complementarity.

`control_scale` applies to direct/target commands, not the integrated velocity
modes or a physical `joint_spool_angle` transmission. Extra cylinder turns do
not model rope self-contact, layered packing, or a changing spool radius.

## Compatibility

Existing MJCF settings keep their defaults. Sensor layouts and actuator
transmission requirements remain unchanged. The per-instance plugin state
has increased from five to eight scalar values to track contraction rate;
rebuild/reload models and do not reuse an old plugin-state checkpoint or
compiled model against this library.

The OPH-specific viewer's UI time budget is a separate Python application
change, not a new core plugin parameter. This source update alone is not a
guarantee of real-time simulation or a particular GUI frame rate.

## Validation

The existing public regression suite and the added asset-free
`tests/test_surface_runtime_cache.py` cover:

- mesh-route rigid-frame invariance and cancellation of internal body forces;
- hard-anchor changes invalidating cached geometry;
- initialization hints remaining irrelevant after initialization;
- reset and independent copies of runtime data;
- body-wrench virtual work against a finite-difference length gradient;
- shared command scaling, actuated velocity friction initialization/stepping,
  cylinder extra turns, and invalid cylinder settings.

```bash
cmake --build build --config Release
# Use .so instead of .dylib on Linux.
CABLE_PLUGIN_LIBRARY="$PWD/build/plugin/libcable_unilateral.dylib" \
  PYTHONPATH=python python -m unittest discover -s tests -v
python scripts/validate_release_tree.py
```

Tests of specific poses and short trajectories do not prove global route
correctness. Automatic topology switching, mass-bearing ropes, arbitrary
mutable geometry, and a GPU backend are not added here. A previously observed
cylinder polyline length-rate inconsistency under some whole-body rigid
motions is also not addressed; mesh invariance is tested separately.
