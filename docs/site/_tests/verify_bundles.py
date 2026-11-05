"""Load the actual extracted downloads, not the original checkout models."""
import argparse
import json
from pathlib import Path
import tempfile
import zipfile

import mujoco
from catalog import CASES

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--plugin", required=True)
args = parser.parse_args()
mujoco.mj_loadPluginLibrary(args.plugin)
downloads = Path(__file__).resolve().parents[1] / "downloads"
records = []
for case in CASES:
    with tempfile.TemporaryDirectory() as folder:
        with zipfile.ZipFile(downloads / f"{case['id']}-model.zip") as package:
            package.extractall(folder)
        model = mujoco.MjModel.from_xml_path(str(Path(folder) / case["model"]))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for _ in range(2):
            mujoco.mj_step(model, data)
        records.append(dict(case=case["id"], nq=model.nq, nu=model.nu, loaded=True))
print(json.dumps(records, indent=2))
