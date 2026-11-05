# Installation and Running

This guide uses Python 3.12, MuJoCo 3.4.0, Git and a C++17 compiler on macOS or Linux. For Windows, follow the [Windows installation tutorial](https://mujocable.github.io/tutorials/windows/).

## 1. Download the Repository

```bash
git clone https://github.com/MuJoCable/mujoco-cable-dynamics.git
cd mujoco-cable-dynamics
```

Run the remaining commands from this directory.

## 2. Create an Environment and Install Dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "mujoco==3.4.0" numpy cmake pillow matplotlib
```

## 3. Build the Plugin

```bash
MUJOCO_DIR="$(python -c 'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release
```

The build produces:

| Platform | Library |
|---|---|
| macOS | `build/plugin/libcable_unilateral.dylib` |
| Linux | `build/plugin/libcable_unilateral.so` |

## 4. Check the Plugin

Set the library location and run a 50-step headless test:

```bash
export CABLE_PLUGIN_LIBRARY="$(python -c 'from pathlib import Path; import sys; print(Path("build/plugin/libcable_unilateral" + (".dylib" if sys.platform == "darwin" else ".so")).resolve())')"
python scripts/smoke_cpp_plugin.py \
  --plugin "$CABLE_PLUGIN_LIBRARY" \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml \
  --steps 50
```

## 5. Open the Viewer and Take Up Cable Manually

```bash
./scripts/run_demo.sh 15 --no-auto-control --show-cable-state --duration 120
```

Expand **Control** on the right and adjust `cable_contraction`. An input of `0.010` means 10 mm of take-up. Close the window to return to the terminal.

## 6. Run an Automatic Demonstration

```bash
./scripts/run_demo.sh 15 \
  --auto-actuator cable_contraction --ctrl-final 0.030 \
  --show-cable-state --duration 120
```

The script updates control during the automatic demonstration. To operate the sliders yourself, use the command in step 5.

## 7. Open Other Examples

```bash
python examples/paper/run.py list
python examples/paper/run.py view rolling_mesh --show-cable-state
python examples/paper/run.py view finger --show-cable-state
python examples/paper/run.py view spirobs --show-cable-state
```

These commands use manual control. Find screenshots, videos, parameters and complete model bundles in the [gallery](gallery.md). Continue to the [quick start](quickstart.md) to build your first model.

## 8. Run Tests (Optional)

```bash
PYTHONPATH=python python -m unittest discover tests
```

## Reopening and Updating

In a new terminal, enter the repository directory and run:

```bash
source .venv/bin/activate
./scripts/run_demo.sh 15 --no-auto-control --show-cable-state
```

After updating source, close the old viewer, repeat step 3, then reopen the model. Prebuilt packages are listed under [Releases](https://github.com/MuJoCable/mujoco-cable-dynamics/releases).
