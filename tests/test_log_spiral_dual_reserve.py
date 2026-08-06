import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos/33_cpp_plugin_log_spiral_dual_reserve.xml"
VARIANTS = ROOT / "cable_plugin_demos/open_spirob_friction_variants"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


class LogSpiralDualReserveTest(unittest.TestCase):
    def test_model_has_two_reserved_guided_cables_and_self_collision(self):
        root = ET.parse(MODEL).getroot()
        compiler = root.find("compiler")
        self.assertEqual(compiler.attrib["meshdir"], "assets/open_spirob")

        instances = root.findall("./extension/plugin/instance")
        self.assertEqual(len(instances), 2)
        for instance in instances:
            configs = {
                node.attrib["key"]: node.attrib["value"]
                for node in instance.findall("config")
            }
            self.assertEqual(configs["route_mode"], "surface")
            self.assertAlmostEqual(float(configs["pretension_offset"]), -0.025)
            self.assertAlmostEqual(float(configs["guide_friction_mu"]), 0.015)
            self.assertEqual(configs["capstan_direction"], "forward")

        sites = {
            site.attrib["name"]: site
            for site in root.findall(".//site")
            if "name" in site.attrib
        }
        for seed in root.findall("./tendon/spatial"):
            route_sites = [node.attrib["site"] for node in seed.findall("site")]
            self.assertEqual(sites[route_sites[0]].attrib["user"], "1")
            self.assertEqual(sites[route_sites[-1]].attrib["user"], "1")
            self.assertTrue(
                all(sites[name].attrib["user"] == "3" for name in route_sites[1:-1])
            )

        pairs = root.findall("./contact/pair")
        self.assertEqual(len(pairs), 11)
        for index, pair in enumerate(pairs, start=1):
            self.assertEqual(pair.attrib["geom1"], f"unit_{index}_geom")
            self.assertEqual(pair.attrib["geom2"], f"unit_{index + 1}_geom")

        variants = sorted(VARIANTS.glob("open_spirob_mu_*.xml"))
        self.assertEqual(len(variants), 5)
        for variant in variants:
            variant_root = ET.parse(variant).getroot()
            self.assertEqual(
                variant_root.find("compiler").attrib["meshdir"],
                "../assets/open_spirob",
            )

    def test_current_plugin_loads_and_steps_model(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN.resolve()))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        data.ctrl[:] = 0.001
        for _ in range(100):
            mujoco.mj_step(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.sensordata).all())
        self.assertGreater(data.ncon, 0)


if __name__ == "__main__":
    unittest.main()
