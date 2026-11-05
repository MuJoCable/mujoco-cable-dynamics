# English Quick Start

MuJoCable is a massless, pull-only cable-transmission plugin for MuJoCo. It computes moving surface routes, axial tension and segment friction, then maps cable forces to rigid bodies. This page is a short English entry point; the complete task-oriented manual is currently in Chinese.

## Install and Build

```bash
git clone https://github.com/MuJoCable/mujoco-cable-dynamics.git
cd mujoco-cable-dynamics
python -m venv .venv
source .venv/bin/activate
python -m pip install 'mujoco==3.4.0' numpy
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$(python -c 'import pathlib,mujoco; print(pathlib.Path(mujoco.__file__).parent)')"
cmake --build build --config Release
```

The library is `build/plugin/libcable_unilateral.dylib` on macOS and `.so` on Linux. Use a matching MuJoCo version and CPU architecture. Precompiled release availability is platform-specific.

## Open and Control

Download [minimal_surface.xml](downloads/minimal_surface.xml) into the repository root:

```bash
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model minimal_surface.xml --no-auto-control --show-cable-state
```

On Linux, replace `mjpython` with `python` and `.dylib` with `.so`. In the Control panel, set `pull=0.010` for 10 mm of take-up, not 0.01 N. Remove `--no-auto-control` and add `--auto-actuator pull --ctrl-final 0.030` for an automatic ramp.

## Modeling Contract

| Element | Meaning |
|---|---|
| `site user="1"` | Physical endpoint attached to its body |
| `site user="2"` | Initialization hint; not a runtime cable node or force application point |
| `site user="3"` | Physical guide that the runtime route must pass through |
| `site user="0"` | Ordinary marker; no cable role |
| `wrap_geoms` | Explicit obstacle list, one entry per role-2 hint in seed order |
| Surface `gear="0"` | Avoid duplicate native-tendon loading; the plugin applies body forces |
| No actuator | A passive surface cable; finite stiffness is not an exact length constraint |

Keep the seed tendon visually hidden: widening it displays a site-to-site polyline, not the computed surface route. The plugin has its own `visual_width` and standard-scene drawing callback.

## Physics and Responsibility

MuJoCo handles rigid-body inertia, joints, contacts, constraints and time integration. The plugin handles the configured cable path and its force map. Cable friction does not come from `geom friction` automatically. `capstan_mu` controls wrap friction; `guide_friction_mu` controls ideal point-guide friction.

The current route class is initialized and retained. There is no general automatic topology switching, massive rope, sag, bending stiffness, knotting, free-rope collision or exact static-stick friction. A point eyelet is not finite-radius cable/hole-wall contact. The experimental `implicit_compliant` interface needs a modified MuJoCo core; it is not supported by an unmodified 3.4.0 binary.

## Examples and Reproduction

[Browse examples](gallery.md) for screenshots, playable raw/annotated videos and dependency-closed model ZIPs. Use the repository wrapper for manual control:

```bash
python -m pip install -r examples/paper/requirements.txt
python examples/paper/run.py view rolling_mesh --show-cable-state
python examples/paper/run.py view spirobs --show-cable-state
python examples/paper/run.py result1 --skip-videos
python examples/paper/run.py result2 --skip-videos
```

Historical videos and fresh numerical reproductions are labeled separately. See [validation](validation.md), [downloads](downloads.md), the [repository](https://github.com/MuJoCable/mujoco-cable-dynamics) and the [paper](https://arxiv.org/abs/2609.09612).
