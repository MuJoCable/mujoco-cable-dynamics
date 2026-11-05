"""Check the downloadable tutorial models against a compiled plugin."""
import argparse
from pathlib import Path
import json

import mujoco
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--plugin", required=True)
args = parser.parse_args()
mujoco.mj_loadPluginLibrary(args.plugin)
folder = Path(__file__).resolve().parents[1] / "downloads"
report = []
for name in ("minimal_surface", "minimal_passive", "minimal_native"):
    model = mujoco.MjModel.from_xml_path(str(folder / f"{name}.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    minimum_tension = float("inf")
    maximum_status = 0
    for step in range(2000):
        if model.nu:
            data.ctrl[0] = min(0.03, step * model.opt.timestep * 0.03)
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        minimum_tension = min(minimum_tension, float(data.sensordata[5]))
        if model.nsensordata == 12:
            maximum_status = max(maximum_status, int(data.sensordata[8]))
    assert minimum_tension >= 0 and maximum_status <= 1
    report.append(dict(model=name, steps=2000, nu=model.nu, sensor_dim=model.nsensordata,
                       minimum_tension_N=minimum_tension, maximum_route_status=maximum_status))
print(json.dumps(report, indent=2))
