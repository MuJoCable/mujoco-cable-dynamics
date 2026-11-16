# Troubleshooting

## A Useful Order

Check mechanism and contacts without cables, then add one cable, modest pretension and a small take-up. Inspect tension and route status before introducing friction or complex control. Change one interpretable factor at a time and preserve XML revisions.

## Symptom Checklist

| Symptom | Check first | Minimal response |
|---|---|---|
| Unknown `plugin:control_timeconstant` or another key | Actual loaded library | Rebuild, exit old viewers, verify `--plugin` |
| `wrap_geoms count...role-2` | Hint count/order versus geoms | Enumerate the seed; do not mark guides as hints |
| Straight lines through site spheres | Native seed is visible | Use a near-zero legal seed width and confirm runtime rendering |
| Unsupported bends or long support rods | Physical meaning of guides | Role-3 points must represent real supports; invented points change moment arms |
| Endpoints appear detached | Parent bodies and transforms | Show local frames; check units and mesh transforms |
| Route takes an end cap or vertical side | Initial corridor and thickness direction | Move sites to the correct section and reinitialize |
| Cable crosses another object | Missing configured obstacle | List relevant wrap surfaces; no automatic scene-wide cable collision |
| Zero length or a route frozen in place | Invalid route or stale visualization | Inspect sensor index 8; do not treat invalid length as real data |
| Take-up produces no motion | Reserve, forces, constraints, control source | Inspect T, c and Lf, then hinges, equalities, ranges and automatic overrides |
| Joint locks | Excess pretension, interference or stiffness | Reduce preload/stiffness; test a minimal ligament pair |
| Jitter or tension jumps | Command jumps, rough surfaces, active-set changes | Reduce input rate, inspect contacts/residuals, then try smoothing/hysteresis |
| Sliders lag | Step cost and GUI synchronization | Measure step, copy and rendering separately |
| Cable remains wrapped after landing | Taut opposite side or fixed branch | Zero tension does not imply simulated sag or automatic lift-off |

## Debug Views

```bash
./scripts/run_demo.sh 25 --show-route-debug --show-cable-state --duration 120
```

The debug layer may show seed order, hints and wrap surfaces together. Not all of these are physical force elements. Disable debug overlays for final presentation and inspect the actual route separately.

Site sphere size, seed width and `visual_width` belong to different layers. Surface `visual_width` is a scene line-width parameter, not cable radius in meters, and does not add collision thickness.

## Diagnostic Logs

At minimum record `time, ctrl, contraction, L, Lf, T, route_status, tangent_residual, surface_residual`, plus relevant body poses, velocities, contacts and joint limits. Count invalid samples separately. Averaging their zeroed tensions into valid samples cannot support a low-friction claim.

For surface actuation inspect `qfrc_passive` and sensors. For native mode inspect actuator transmission and corrections. Ideal baselines may also impose equality constraints. Label direct-qpos playback separately from cable-driven dynamics.

## Performance Is Not Stability

Rigid-frame route caching, visibility memoization and body-wrench accumulation reduce repeated work, but do not prove arbitrary meshes run in real time. A smaller GUI compute budget can improve interaction without making physical simulation faster.

Do not increase timestep solely to make the viewer feel responsive. First check convergence of pose, tension, energy and contacts. Display smoothing cannot correct physical route, material or collision errors.
