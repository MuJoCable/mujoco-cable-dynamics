import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from scripts.analyze_free_rotating_pulley import simulate


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos/29_cpp_plugin_free_rotating_pulley.xml"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


class FreeRotatingPulleyTest(unittest.TestCase):
    def test_model_has_inertial_hinge_and_velocity_directed_friction(self):
        root = ET.parse(MODEL).getroot()
        instance = root.find(
            "./extension/plugin/instance[@name='free_pulley_cable']"
        )
        self.assertIsNotNone(instance)
        configs = {
            node.attrib["key"]: node.attrib["value"]
            for node in instance.findall("config")
        }
        self.assertEqual(configs["route_mode"], "surface")
        self.assertEqual(configs["route_tendon"], "free_pulley_seed")
        self.assertEqual(configs["wrap_geoms"], "free_pulley_wrap")
        self.assertEqual(configs["capstan_direction"], "velocity")
        self.assertGreater(float(configs["capstan_mu"]), 0)
        self.assertGreater(float(configs["capstan_velocity_scale"]), 0)

        pulley = root.find("./worldbody/body[@name='free_pulley']")
        self.assertIsNotNone(pulley)
        hinge = pulley.find("./joint[@name='pulley_hinge']")
        self.assertIsNotNone(hinge)
        self.assertEqual(hinge.attrib["type"], "hinge")
        wrap = pulley.find("./geom[@name='free_pulley_wrap']")
        self.assertIsNotNone(wrap)
        self.assertGreater(float(wrap.attrib["mass"]), 0)
        self.assertIsNone(root.find("./actuator"))

        seed = root.find("./tendon/spatial[@name='free_pulley_seed']")
        self.assertIsNotNone(seed)
        self.assertEqual(
            [node.attrib["site"] for node in seed.findall("site")],
            ["left_cable_end", "pulley_upper_hint", "right_cable_end"],
        )

    def test_dynamic_balance_and_dissipation(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        _, summary = simulate(
            DEFAULT_PLUGIN, MODEL, duration=0.34, sample_stride=10
        )
        self.assertTrue(summary["pass"], summary)
        self.assertGreater(summary["peak_tension_difference_N"], 0.01)
        self.assertGreater(
            summary["peak_pulley_angular_velocity_rad_s"], 1.0
        )
        self.assertGreaterEqual(summary["minimum_dissipated_power_W"], 0)


if __name__ == "__main__":
    unittest.main()
