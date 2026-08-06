# Running All Local Demos

Run every command from the repository root. Build once so all plugin examples
use the current source tree rather than an older library.

## Build

```bash
python -m pip install -e .
MUJOCO_DIR="$(python -c 'import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release

# macOS
export MUJOCABLE_PLUGIN="$PWD/build/plugin/libcable_unilateral.dylib"
# Linux: export MUJOCABLE_PLUGIN="$PWD/build/plugin/libcable_unilateral.so"
```

The launcher selects `mjpython` on macOS and `python3` on Linux.

## Pulley, Winch, and Eyelet Models

```bash
./scripts/run_demo.sh 09 --show-route-debug --duration 120
./scripts/run_demo.sh 10 --show-route-debug --duration 120
./scripts/run_demo.sh 11 --show-route-debug --duration 120
./scripts/run_demo.sh 12 --show-route-debug --duration 120
./scripts/run_demo.sh 15 --show-route-debug --duration 120
./scripts/run_demo.sh 21 --show-route-debug --duration 120
./scripts/run_demo.sh 29 --show-route-debug --duration 120
./scripts/run_demo.sh 31 --duration 120
./scripts/run_demo.sh 32 --duration 120
```

Demo 31 uses native rigid-flex aperture collision. Demo 32 uses an ideal
role-3 eyelet with local Capstan friction.

## Joints, Hands, and Tensegrity

```bash
./scripts/run_demo.sh 13 --show-route-debug --duration 120
./scripts/run_demo.sh 14 --show-route-debug --duration 120
./scripts/run_demo.sh 16 --show-route-debug --duration 120
./scripts/run_demo.sh 20 --show-route-debug --duration 120
./scripts/run_demo.sh 17 --duration 120
./scripts/run_demo.sh 18 --duration 120
./scripts/run_demo.sh 19 --duration 120
./scripts/run_demo.sh 24 --duration 120
./scripts/run_demo.sh 25 --show-route-debug --duration 120
./scripts/run_demo.sh 26 --show-route-debug --duration 120
./scripts/run_demo.sh 27 --show-route-debug --duration 120
./scripts/run_demo.sh 28 --show-route-debug --duration 120
```

Demos 18 and 24 are the native-tendon and virtual-hinge baselines. Demo 16 is
passive; drag the distal body in the viewer to inspect compliant recovery.

## Log-Spiral Robot

Automatic common take-up followed by antagonistic differential payout:

```bash
./scripts/run_demo.sh 33 \
  --mode differential --reserve 0.025 --contraction 0.050 \
  --ramp-time 4 --period 12 --duration 120
```

Manual control:

```bash
./scripts/run_demo.sh 33 --mode manual --duration 120
```

The five friction variants are under
`cable_plugin_demos/open_spirob_friction_variants/`. For example:

```bash
mjpython scripts/view_log_spiral_dual_reserve.py \
  --plugin "$MUJOCABLE_PLUGIN" \
  --model cable_plugin_demos/open_spirob_friction_variants/open_spirob_mu_0p100.xml \
  --mode differential --reserve 0.025 --contraction 0.050 \
  --ramp-time 4 --period 12 --duration 120
```

Replace `0p100` with `0p000`, `0p015`, `0p050`, or `0p200` for the
other friction coefficients.

## Physics-Boundary Model and Validation

```bash
./scripts/run_demo.sh 30 --case sliding --duration 120
./scripts/run_demo.sh 30 --case stick --duration 120

python scripts/validate_release_tree.py
CABLE_PLUGIN_LIBRARY="$MUJOCABLE_PLUGIN" PYTHONPATH=python \
  python -m unittest discover tests
python scripts/smoke_cpp_plugin.py \
  --plugin "$MUJOCABLE_PLUGIN" \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml
python scripts/check_log_spiral_dual_reserve.py \
  --plugin "$MUJOCABLE_PLUGIN"
```

Demo 30 is a separate massive-rope transport benchmark and does not claim that
the present massless cable plugin models rope material transport.
