# Sensors and Python

## Register a Sensor

```xml
<sensor><plugin name="rope_state" instance="rope"/></sensor>
```

Native mode outputs eight values; surface mode outputs twelve. Omitting the sensor does not disable cable forces, but you cannot read that instance's outputs from `sensordata` without one.

| Index | Name | Unit | Meaning |
|---:|---|---|---|
| 0 | length | m | Solved runtime route length |
| 1 | velocity | m/s | Geometric length rate |
| 2 | free_length | m | Constitutive free length, excluding the slack dead zone |
| 3 | contraction | m | Actual mapped contraction after any smoothing/integration |
| 4 | extension | m | `length - free_length - slack` |
| 5 | tension | N | Source constitutive tension, not every segment's tension |
| 6 | taut | 0/1 | Positive-tension state |
| 7 | saturated | 0/1 | Source tension reached its cap |
| 8 | route_status | enum | 0 valid, 1 degraded, 2 invalid |
| 9 | tangent_residual | solver metric | Active-path tangency/optimization residual |
| 10 | surface_residual | m | Surface/penetration-related geometric residual |
| 11 | solver_iterations | count | Iteration count; caching can reduce it |

`tangent_residual` is not a universal angular error in degrees. Interpret backend-specific residuals according to their implementation, not by comparing them directly with a distance in meters.

## Read by Name

```python
import numpy as np
import mujoco

sid = model.sensor("rope_state").id
adr = int(model.sensor_adr[sid])
dim = int(model.sensor_dim[sid])
values = np.asarray(data.sensordata[adr:adr + dim]).copy()
length_m, length_rate, free_m = values[:3]
contraction_m, extension_m, tension_N = values[3:6]
if dim == 12 and values[8] > 1:
    raise RuntimeError("invalid cable route; inspect geometry before continuing")
```

The copy prevents later steps from overwriting your record. Call at least `mj_forward` before reading. Dynamic logs are commonly sampled after `mj_step`.

## Record CSV

```python
import csv

with open("cable.csv", "w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["time_s", "length_m", "free_length_m", "contraction_m", "tension_N"])
    for step in range(5000):
        mujoco.mj_step(model, data)
        x = data.sensordata[adr:adr + dim]
        if step % 20 == 0:
            writer.writerow([data.time, x[0], x[2], x[3], x[5]])
```

Also preserve model revision, library SHA-256, timestep, controller, friction, reserve and units. The paper wrapper separates new outputs from historical evidence.

## Viewer Readouts and Slack Colors

`--show-cable-state` displays length, reference length, strain and tension labels. Slack coloring can turn zero-tension cables gray; `--no-slack-color` preserves topology colors. Check `--help` for your script version instead of mixing options from specialized launchers.

In surface mode `actuator_force` reports `-T`, while actual forces enter `qfrc_passive`. Looking only at `qfrc_actuator` therefore does not establish whether a cable drives the mechanism. Native mode uses the MuJoCo actuator transmission for its principal force, plus plugin corrections.

The twelve-value sensor does not contain the full contact-point or segment-tension arrays. Use case-analysis scripts, known propagation relations or additional diagnostics for individual segments; do not copy source tension into every segment.

## Invalid Routes

An invalid step disables cable force and zeros some constitutive outputs. `length=0` then does not mean the physical cable shrank to zero length. Check route status and warnings before averaging such samples. Sensor evaluation does not silently recompute a successful route to conceal the preceding force-stage failure.
