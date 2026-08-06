import json
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos" / "16_cpp_plugin_passive_saddle_joint.xml"
ASSET_DIR = ROOT / "cable_plugin_demos" / "assets" / "passive_saddle_joint"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)
INSTANCE_NAMES = {
    "passive_figure_eight_upper",
    "passive_figure_eight_lower",
}
SENSOR_NAMES = tuple(f"{name}_state" for name in INSTANCE_NAMES)


class PassiveSaddleAssetTest(unittest.TestCase):
    def test_source_and_generated_mesh_roles_are_explicit(self):
        manifest = json.loads((ASSET_DIR / "asset_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(len(manifest["sources"]), 2)
        for source in manifest["sources"]:
            self.assertTrue(source["closed"])
            self.assertTrue(source["manifold"])
            self.assertFalse(source["convex_local_test"])
            self.assertGreater(source["faces"], 10_000)
            self.assertTrue((ASSET_DIR / source["file"]).exists())
            self.assertTrue((ASSET_DIR / source["source_file"]).exists())
            self.assertIn("voxel remesh", source["repair"]["method"])
            self.assertTrue(source["source_topology"]["closed"])
            self.assertTrue(source["source_topology"]["manifold"])

        for side in ("proximal", "distal"):
            contact = manifest["generated"][side]["contact_patch"]
            self.assertLessEqual(contact["faces"], 600)
            self.assertEqual(contact["degenerate_faces"], 0)
            self.assertTrue(contact["manifold"])
            self.assertNotIn("route_proxy", manifest["generated"][side])

        self.assertFalse((ASSET_DIR / "proximal_route_proxy.obj").exists())
        self.assertFalse((ASSET_DIR / "distal_route_proxy.obj").exists())

        root = ET.parse(MODEL).getroot()
        mesh_files = {
            mesh.attrib["file"] for mesh in root.findall("./asset/mesh")
        }
        self.assertEqual(
            mesh_files, {"test_1_routed.stl", "test_2_routed.stl"}
        )


class PassiveSaddleModelTest(unittest.TestCase):
    def test_xml_has_only_passive_free_body_and_two_surface_ligaments(self):
        root = ET.parse(MODEL).getroot()
        self.assertEqual(len(root.findall(".//freejoint")), 1)
        self.assertEqual(root.findall(".//joint"), [])
        self.assertEqual(root.findall(".//equality"), [])
        self.assertEqual(root.findall(".//actuator"), [])

        instances = root.findall("./extension/plugin/instance")
        self.assertEqual({item.attrib["name"] for item in instances}, INSTANCE_NAMES)
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
            self.assertEqual(config["home_length"], "auto_initial")
            self.assertEqual(float(config["stiffness"]), 400.0)
            self.assertEqual(float(config["damping"]), 2.0)
            self.assertEqual(float(config["slack"]), 0.0002)
            self.assertEqual(float(config["pretension_offset"]), 0.001575)
            self.assertEqual(float(config["taut_transition"]), 0.00015)
            self.assertEqual(float(config["taut_hysteresis"]), 0.00005)
            self.assertEqual(float(config["route_hysteresis"]), 0.00002)
            self.assertEqual(
                float(config["visual_smoothing_timeconstant"]), 0.025
            )
            self.assertEqual(float(config["max_tension"]), 3.0)

        endpoint_sites = root.findall('.//site[@user="1"]')
        hint_sites = root.findall('.//site[@class="route_hint"]')
        hint_default = root.find('.//default[@class="route_hint"]/site')
        self.assertEqual(len(endpoint_sites), 4)
        self.assertEqual(len(hint_sites), 4)
        self.assertIsNotNone(hint_default)
        self.assertEqual(hint_default.attrib["user"], "2")

    def test_passive_plugin_generates_force_without_actuator(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN))
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertEqual(model.nu, 0)
        self.assertEqual(model.neq, 0)
        self.assertEqual(model.nq, 7)
        self.assertEqual(model.nv, 6)
        self.assertGreaterEqual(data.ncon, 2)
        self.assertLessEqual(data.ncon, 32)
        self.assertGreater(float(np.linalg.norm(data.qfrc_passive)), 0.005)

        for name in SENSOR_NAMES:
            sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            self.assertGreaterEqual(sensor_id, 0)
            address = int(model.sensor_adr[sensor_id])
            dimension = int(model.sensor_dim[sensor_id])
            self.assertEqual(dimension, 12)
            values = np.asarray(data.sensordata[address : address + dimension], dtype=float)
            self.assertAlmostEqual(float(values[3]), 0.0, places=12)
            self.assertAlmostEqual(float(values[5]), 0.52, places=7)
            self.assertEqual(int(round(float(values[8]))), 0)
            self.assertLess(float(values[9]), 1e-5)
            self.assertLess(float(values[10]), 1e-7)

        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "distal_metacarpal")
        initial_position = np.asarray(data.xpos[body_id], dtype=float).copy()
        maximum_contacts = int(data.ncon)
        for _ in range(20):
            mujoco.mj_step(model, data)
            maximum_contacts = max(maximum_contacts, int(data.ncon))
        displacement = float(np.linalg.norm(np.asarray(data.xpos[body_id]) - initial_position))
        self.assertLessEqual(maximum_contacts, 128)
        self.assertLess(displacement, 1e-3)
        for name in SENSOR_NAMES:
            sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            address = int(model.sensor_adr[sensor_id])
            self.assertLessEqual(int(round(float(data.sensordata[address + 8]))), 1)


if __name__ == "__main__":
    unittest.main()
