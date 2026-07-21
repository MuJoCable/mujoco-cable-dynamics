import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos" / "20_cpp_plugin_controlled_saddle_joint.xml"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


class ControlledSaddleModelTest(unittest.TestCase):
    def test_xml_has_two_passive_and_two_control_cables(self):
        root = ET.parse(MODEL).getroot()
        self.assertEqual(len(root.findall(".//freejoint")), 1)
        self.assertEqual(root.findall(".//joint"), [])
        self.assertEqual(root.findall(".//equality"), [])

        instances = root.findall("./extension/plugin/instance")
        self.assertEqual(len(instances), 4)
        for instance in instances:
            config = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }
            self.assertEqual(config["route_mode"], "surface")
            self.assertEqual(config["mesh_route_mode"], "taut_obstacle")
            self.assertEqual(
                config["wrap_geoms"],
                "proximal_saddle_visual distal_saddle_visual",
            )
            self.assertEqual(float(config["taut_transition"]), 0.00015)
            self.assertEqual(float(config["taut_hysteresis"]), 0.00005)
            self.assertEqual(float(config["route_hysteresis"]), 0.00002)
            self.assertEqual(
                float(config["visual_smoothing_timeconstant"]), 0.025
            )

        for instance in instances[2:]:
            config = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }
            self.assertEqual(float(config["control_timeconstant"]), 0.25)
            self.assertEqual(float(config["max_contraction_rate"]), 0.003)

        actuators = root.findall("./actuator/plugin")
        self.assertEqual(
            {actuator.attrib["name"] for actuator in actuators},
            {"saddle_upper_control", "saddle_lower_control"},
        )
        self.assertTrue(all(actuator.attrib["gear"] == "0" for actuator in actuators))
        self.assertTrue(
            all(actuator.attrib["ctrlrange"] == "-0.006 0.006" for actuator in actuators)
        )

        tendon_sites = {
            tendon.attrib["name"]: [site.attrib["site"] for site in tendon]
            for tendon in root.findall("./tendon/spatial")
        }
        self.assertEqual(
            tendon_sites["upper_control_seed"],
            [
                "figure8_prox_upper",
                "figure8_prox_hint_upper",
                "figure8_dist_hint_upper",
                "figure8_dist_upper",
            ],
        )
        self.assertEqual(
            tendon_sites["lower_control_seed"],
            [
                "figure8_prox_lower",
                "figure8_prox_hint_lower",
                "figure8_dist_hint_lower",
                "figure8_dist_lower",
            ],
        )

    def test_control_routes_initialize_on_actual_meshes(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertEqual(model.nu, 2)
        self.assertEqual(model.neq, 0)
        for sensor_name in (
            "passive_figure_eight_upper_state",
            "passive_figure_eight_lower_state",
            "upper_control_state",
            "lower_control_state",
        ):
            sensor_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name
            )
            address = int(model.sensor_adr[sensor_id])
            values = np.asarray(data.sensordata[address : address + 12])
            self.assertEqual(int(round(float(values[8]))), 0)
            self.assertLess(float(values[9]), 1e-5)
            self.assertLess(float(values[10]), 1e-7)

    def test_coupled_mesh_routes_remain_valid_during_initial_settling(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        sensor_addresses = []
        for sensor_name in (
            "passive_figure_eight_upper_state",
            "passive_figure_eight_lower_state",
            "upper_control_state",
            "lower_control_state",
        ):
            sensor_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name
            )
            sensor_addresses.append(int(model.sensor_adr[sensor_id]))

        for _ in range(240):
            mujoco.mj_step(model, data)
            self.assertTrue(
                all(
                    int(round(float(data.sensordata[address + 8]))) <= 1
                    for address in sensor_addresses
                )
            )


if __name__ == "__main__":
    unittest.main()
