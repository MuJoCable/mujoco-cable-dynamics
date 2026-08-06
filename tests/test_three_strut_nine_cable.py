import importlib.util
import os
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos" / "17_cpp_plugin_three_strut_nine_cable.xml"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


def load_checker():
    path = ROOT / "scripts" / "check_three_strut_nine_cable.py"
    spec = importlib.util.spec_from_file_location("check_three_strut_nine_cable", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load script module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ThreeStrutNineCableTest(unittest.TestCase):
    def test_xml_is_three_free_struts_and_nine_plugin_cables(self):
        root = ET.parse(MODEL).getroot()
        self.assertEqual(len(root.findall(".//body/joint[@type='free']")), 3)
        self.assertEqual(root.findall(".//equality"), [])
        self.assertEqual(len(root.findall("./tendon/spatial")), 9)
        self.assertEqual(len(root.findall("./actuator/plugin")), 9)
        self.assertEqual(len(root.findall("./sensor/plugin")), 9)
        self.assertIsNotNone(root.find(".//geom[@name='grid_ground']"))
        self.assertLess(float(root.find("./option").attrib["gravity"].split()[2]), -9.0)

        instances = root.findall("./extension/plugin/instance")
        self.assertEqual(len(instances), 9)
        offsets = []
        for instance in instances:
            config = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }
            self.assertEqual(config["route_mode"], "native")
            self.assertEqual(config["home_length"], "auto_initial")
            self.assertEqual(float(config["max_tension"]), 80.0)
            self.assertEqual(float(config["control_timeconstant"]), 0.04)
            self.assertEqual(float(config["max_contraction_rate"]), 0.025)
            offsets.append(float(config["pretension_offset"]))
        self.assertEqual(offsets[:6], [0.00096810] * 6)
        self.assertEqual(offsets[6:], [0.002] * 3)

    def test_self_stress_is_balanced_and_recovers_after_large_contraction(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        report = load_checker().analyze_demo(plugin_path=DEFAULT_PLUGIN)
        self.assertTrue(report["pass"], report)
        self.assertAlmostEqual(
            report["initial"]["ring_to_cross_tension_ratio"], 0.48405, places=5
        )
        self.assertGreater(
            report["contraction_cycle"]["maximum_pairwise_shape_change_m"], 0.03
        )
        self.assertLess(
            report["contraction_cycle"]["final_cable_length_error_m"], 1e-4
        )


if __name__ == "__main__":
    unittest.main()
