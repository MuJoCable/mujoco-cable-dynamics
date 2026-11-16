# Your First Cable Model

This example uses a fixed pulley and a payload with a `freejoint`, not a vertical slide. Gravity, cable forces and any enabled rigid-body contacts determine its motion.

## 1. Download the Model

Save <a href="../downloads/minimal_surface.xml">minimal_surface.xml</a> in the repository root. It uses primitive geometry and needs no STL.

```{literalinclude} downloads/minimal_surface.xml
:language: xml
:lines: 1-24
```

`start` is the fixed endpoint, `hint` selects the upper wrapping side, `end` is attached to the payload, and `pulley_wrap` is the obstacle. After initialization the hint is no longer a length or force node.

## 2. Control Take-Up Manually

```bash
# macOS; on Linux use python and the .so suffix.
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model minimal_surface.xml \
  --no-auto-control --show-cable-state --duration 120
```

Adjust `pull` in the viewer's **Control** panel:

| Input | Meaning |
|---|---|
| `0` | No extra take-up; configured pretension remains |
| `0.010` | Reduce the free-length target by 10 mm |
| `0.030` | Take up 30 mm, not 0.03 N or a prescribed 30 mm payload displacement |

Contraction $c$ is positive for take-up. Geometric length $L$, free length $L_f$ and tension $T$ are different quantities. The payload response follows from dynamics.

## 3. Run an Automatic Ramp

```bash
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model minimal_surface.xml \
  --auto-actuator pull --ctrl-final 0.030 \
  --ctrl-ramp-start 0.5 --ctrl-ramp-end 2.5 \
  --show-cable-state --duration 120
```

Disable automatic control before dragging a slider, or the controller overwrites its value on the next step.

## 4. Why the Gear Is Zero

```xml
<actuator>
  <plugin name="pull" instance="rope" tendon="seed"
          gear="0" ctrllimited="true" ctrlrange="-0.02 0.06"/>
</actuator>
<sensor><plugin name="rope_state" instance="rope"/></sensor>
```

Surface mode directly maps forces from the solved route to bodies. A second force along the native seed would double-load the system, so the transmission must use `gear="0"`. This does not disable actuation: the input enters the plugin and forces are applied during the passive stage.

## 5. Passive and Native Variants

- <a href="../downloads/minimal_passive.xml">Passive surface XML</a>: no actuator. Perturb the payload and observe tensile extension and recovery. Finite stiffness is not an exact length constraint.
- <a href="../downloads/minimal_native.xml">Native XML</a>: a two-site native path with the plugin's unilateral law. It uses normal tendon transmission, not surface-mode zero gear.

All three XMLs have been loaded and short-run dynamically. Continue with [model structure](modeling.md) and [actuation](actuation.md).
