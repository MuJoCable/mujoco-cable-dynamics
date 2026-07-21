import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from scripts.check_wheel_axle_winding_geometry import analyze as analyze_winding


ROOT = Path(__file__).resolve().parents[1]
MODEL = (
    ROOT
    / "cable_plugin_demos"
    / "21_cpp_plugin_wheel_axle_force_amplifier.xml"
)
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


class WheelAxleForceAmplifierTest(unittest.TestCase):
    def test_xml_uses_shared_shaft_and_opposite_winding(self):
        root = ET.parse(MODEL).getroot()
        instances = {
            instance.attrib["name"]: {
                config.attrib["key"]: config.attrib["value"]
                for config in instance.findall("config")
            }
            for instance in root.findall("./extension/plugin/instance")
        }
        self.assertEqual(
            set(instances),
            {"large_drum_input_cable", "small_drum_output_cable"},
        )
        input_config = instances["large_drum_input_cable"]
        output_config = instances["small_drum_output_cable"]
        self.assertEqual(
            input_config["spool_joint"], "compound_drum_hinge"
        )
        self.assertEqual(
            output_config["spool_joint"], "compound_drum_hinge"
        )
        self.assertEqual(input_config["spool_reserve_direction"], "positive")
        self.assertEqual(output_config["spool_reserve_direction"], "negative")
        self.assertEqual(input_config["spool_reaction_torque"], "true")
        self.assertEqual(output_config["spool_reaction_torque"], "true")
        self.assertAlmostEqual(
            float(input_config["spool_radius"])
            / float(output_config["spool_radius"]),
            3.0,
        )
        debug_entries = {
            item.attrib["name"]: item.attrib["data"].split()
            for item in root.findall("./custom/text")
        }
        self.assertEqual(
            debug_entries["spool_debug_large"],
            [
                "large_drum_input_cable",
                "large_input_drum",
                "large_drum_fixed",
                "large_drum_exit",
                "large_drum_input",
            ],
        )
        self.assertEqual(
            debug_entries["spool_debug_small"],
            [
                "small_drum_output_cable",
                "small_output_drum",
                "small_drum_fixed",
                "small_drum_exit",
                "small_drum_output",
            ],
        )
        drum = root.find("./worldbody/body[@name='compound_drum']")
        self.assertIsNotNone(drum)
        self.assertIsNotNone(drum.find("./site[@name='large_drum_fixed']"))
        self.assertIsNotNone(drum.find("./site[@name='small_drum_fixed']"))
        self.assertIsNotNone(
            root.find("./worldbody/site[@name='large_drum_exit']")
        )
        self.assertIsNotNone(
            root.find("./worldbody/site[@name='small_drum_exit']")
        )
        routes = {
            tendon.attrib["name"]: [
                item.attrib["site"] for item in tendon.findall("site")
            ]
            for tendon in root.findall("./tendon/spatial")
        }
        self.assertEqual(
            routes["large_drum_input"], ["large_drum_exit", "input_hook"]
        )
        self.assertEqual(
            routes["small_drum_output"], ["small_drum_exit", "output_hook"]
        )

    def test_winding_debug_geometry_matches_drum_and_exit(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        report = analyze_winding(DEFAULT_PLUGIN, MODEL)
        self.assertTrue(report["pass"], report)
        self.assertEqual(len(report["spools"]), 2)
        for spool in report["spools"]:
            self.assertLess(spool["phase_error_m"], 1e-6)
            self.assertLess(spool["dynamic_phase_error_max_m"], 1e-6)
            self.assertLess(spool["tangent_residual"], 1e-6)

    def test_static_tension_ratio_matches_radius_ratio(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        for _ in range(2500):
            mujoco.mj_step(model, data)

        tensions = []
        for sensor_name in (
            "large_drum_input_state",
            "small_drum_output_state",
        ):
            sensor_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name
            )
            address = int(model.sensor_adr[sensor_id])
            values = np.asarray(
                data.sensordata[address : address + 8], dtype=float
            )
            self.assertEqual(int(round(float(values[6]))), 1)
            tensions.append(float(values[5]))

        self.assertGreater(tensions[0], 0)
        self.assertAlmostEqual(tensions[1] / tensions[0], 3.0, delta=0.06)


if __name__ == "__main__":
    unittest.main()
