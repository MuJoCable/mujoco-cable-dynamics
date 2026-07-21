import importlib.util
import os
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos" / "19_cpp_plugin_mixed_stiffness_tensegrity.xml"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


def load_script(filename: str):
    path = ROOT / "scripts" / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load script module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class MixedStiffnessTensegrityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.viewer = load_script("view_cpp_plugin_demo.py")
        cls.calculator = load_script("cable_material_calculator.py")

    def test_xml_separates_elastic_ring_and_stiff_cross_cables(self):
        root = ET.parse(MODEL).getroot()
        instances = {}
        for instance in root.findall("./extension/plugin/instance"):
            instances[instance.attrib["name"]] = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }

        ring = [instances[f"cable_{name}"] for name in (
            "bottom_01", "bottom_12", "bottom_20", "top_01", "top_12", "top_20"
        )]
        cross = [instances[f"cable_cross_{index}{index}"] for index in range(3)]
        self.assertEqual([float(item["stiffness"]) for item in ring], [300.0] * 6)
        self.assertEqual([float(item["stiffness"]) for item in cross], [10000.0] * 3)
        self.assertEqual(
            root.find("./actuator/plugin[@name='act_cross_00']").attrib["ctrlrange"],
            "-0.04 0.06",
        )

    def test_ea_over_length_material_conversion(self):
        ring = self.calculator.cable_material_properties(
            young_modulus_pa=53e6,
            diameter_m=1.5e-3,
            reference_length_m=0.31177,
            tension_n=1.45215,
            poisson_ratio=0.45,
        )
        cross = self.calculator.cable_material_properties(
            young_modulus_pa=18.9e9,
            diameter_m=0.5e-3,
            reference_length_m=0.37186,
            tension_n=3.0,
            poisson_ratio=0.3,
        )
        self.assertAlmostEqual(ring["plugin_stiffness_n_per_m"], 300.4, delta=1.0)
        self.assertAlmostEqual(cross["plugin_stiffness_n_per_m"], 9980.0, delta=50.0)
        self.assertLess(cross["axial_strain"], 0.001)
        self.assertLess(ring["diameter_change_m"], 0.0)

    def test_negative_cross_command_is_slack_and_changes_color(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN.resolve()))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        controlled = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_cross_00"
        )
        data.ctrl[controlled] = -0.01
        mujoco.mj_forward(model, data)

        bindings = self.viewer.cable_sensor_bindings(model, MODEL)
        states = self.viewer.read_cable_visual_states(data, bindings)
        indexed = {state.binding.name: state for state in states}
        controlled_state = indexed["cross_00"]
        self.assertFalse(controlled_state.taut)
        self.assertAlmostEqual(controlled_state.tension, 0.0, places=12)
        self.assertLess(controlled_state.extension, 0.0)
        self.assertTrue(all(indexed[name].taut for name in indexed if name != "cross_00"))

        base_rgba = np.asarray(model.tendon_rgba, dtype=float).copy()
        self.viewer.update_cable_state_colors(model, states, base_rgba)
        self.assertTrue(
            np.allclose(model.tendon_rgba[controlled_state.binding.tendon_id], self.viewer.SLACK_RGBA)
        )

        scene = mujoco.MjvScene(model, 100)
        self.viewer.draw_cable_state_labels(model, data, scene, states)
        labels = [scene.geoms[index].label for index in range(scene.ngeom)]
        self.assertTrue(any("cross_00" in label and "SLACK" in label for label in labels))
        self.assertTrue(any("eps=" in label and "T=" in label for label in labels))


if __name__ == "__main__":
    unittest.main()
