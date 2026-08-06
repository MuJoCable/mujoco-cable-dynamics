# Selected Demonstrations

The repository intentionally includes only the models presented on the
[MuJoCable project page](https://mujocable.github.io/).

| Demo | Model | Capability |
|---:|---|---|
| 09 | `09_cpp_plugin_free_hanging_single_pulley.xml` | Physical drum, fixed pulley, free payload |
| 10 | `10_cpp_plugin_dual_pulley_free_payload.xml` | Three-cylinder compound tangent route |
| 11 | `11_cpp_plugin_reverse_reserve_release.xml` | Signed reserve and reverse payout |
| 12 | `12_cpp_plugin_frictional_pulley_free_payload.xml` | Capstan tension propagation |
| 13 | `13_cpp_plugin_rolling_joint_figure_eight.xml` | Cylinder rolling-joint surrogate |
| 14 | `14_cpp_plugin_convex_mesh_rolling_joint.xml` | Convex-mesh surface routing |
| 15 | `15_cpp_plugin_surface_single_pulley.xml` | Runtime cylinder envelope |
| 16 | `16_cpp_plugin_passive_saddle_joint.xml` | Passive nonconvex saddle joint |
| 17 | `17_cpp_plugin_three_strut_nine_cable.xml` | Three-strut nine-cable tensegrity |
| 18 | `18_native_tendon_three_strut_nine_cable.xml` | Matched native-tendon baseline |
| 19 | `19_cpp_plugin_mixed_stiffness_tensegrity.xml` | Mixed stiffness and visible slack state |
| 20 | `20_cpp_plugin_controlled_saddle_joint.xml` | Antagonistically controlled saddle joint |
| 21 | `21_cpp_plugin_wheel_axle_force_amplifier.xml` | Shared-shaft wheel-and-axle transmission |
| 24 | `24_faive_index_pip_virtual_hinge_baseline.xml` | Faive two-virtual-hinge baseline |
| 25 | `25_faive_index_pip_surface_cable.xml` | Faive free-body surface-cable model |
| 26 | `26_cpp_plugin_100_fingers_human_hand.xml` | 100_fingers human hand with five MuJoCable routes per digit |
| 27 | `27_cpp_plugin_100_fingers_index_cable_debug.xml` | Full hand with only the five index-finger cables and route-debug camera |
| 28 | `28_cpp_plugin_100_fingers_index_mesh_threading.xml` | Actual-mesh index threading with five active tendons and six passive cable ligaments |
| 29 | `29_cpp_plugin_free_rotating_pulley.xml` | Velocity-directed cable friction on a free pulley with explicit rotational inertia |
| 30 | `30_cpho_2018_problem3_massive_rope.xml` | CPhO 2018 massive-rope continuum benchmark and current plugin boundary; future design archived in `MASSIVE_CABLE_FUTURE_EXTENSION.md` |
| 31 | `31_rigid_flex_through_hole.xml` | Native mesh convex-hull collision versus a true nonconvex rigid-flex aperture |
| 32 | `32_cpp_plugin_eyelet_friction.xml` | Role-3 analytic eyelet with guide-local Capstan tension transmission |
| 33 | `33_cpp_plugin_log_spiral_dual_reserve.xml` | Log-spiral robot with dual reserve, differential payout, module self-collision, and eyelet friction |

Run any entry after building or unpacking a binary release:

```bash
./scripts/run_demo.sh 15 --show-route-debug --duration 120
```

Demo 30 uses its dedicated massive-rope continuum viewer and does not require
the plugin binary:

```bash
./scripts/run_demo.sh 30 --case sliding --duration 120
./scripts/run_demo.sh 30 --case stick --duration 120
```

Demos 31-32 compare true aperture collision and reduced-order eyelet friction:

```bash
./scripts/run_demo.sh 31 --duration 120
./scripts/run_demo.sh 32 --duration 120
```

Demo 33 provides automatic differential control and manual control:

```bash
./scripts/run_demo.sh 33 --mode differential --reserve 0.025 \
  --contraction 0.050 --ramp-time 4 --period 12 --duration 120
./scripts/run_demo.sh 33 --mode manual --duration 120
```

On macOS the launcher uses `mjpython`; on Linux it uses `python3`. Override the
runner with `MUJOCO_PYTHON` and the library with `MUJOCABLE_PLUGIN`.
See [all commands](RUN_ALL_DEMOS.md) and the
[threading guide](CABLE_DESIGN_AND_THREADING_GUIDE.md) for model-specific
details.
