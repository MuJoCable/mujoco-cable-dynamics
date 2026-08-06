import os
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos/26_cpp_plugin_100_fingers_human_hand.xml"
DEBUG_MODEL = (
    ROOT / "cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml"
)
MESH_MODEL = (
    ROOT
    / "cable_plugin_demos/28_cpp_plugin_100_fingers_index_mesh_threading.xml"
)
ASSET_DIR = ROOT / "cable_plugin_demos/assets/100_fingers_human"
PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)
DIGITS = ("index", "middle", "ring", "little", "thumb")
CABLES = (
    "extensor", "abductor", "adductor",
    "flexor_intermediate", "flexor_distal",
)


class HumanHandStructureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if PLUGIN.exists():
            mujoco.mj_loadPluginLibrary(str(PLUGIN.resolve()))

    def test_assets_are_portable_and_attributed(self):
        manifest = (ASSET_DIR / "asset_manifest.json").read_text(encoding="utf-8")
        self.assertIn('"source_license": "CC BY 4.0"', manifest)
        self.assertNotIn(str(Path.home()), manifest)
        self.assertTrue((ASSET_DIR / "README.md").is_file())
        self.assertEqual(len(list(ASSET_DIR.glob("*.stl"))), 16)

    def test_five_functional_cables_per_digit(self):
        root = ET.parse(MODEL).getroot()
        instances = root.findall("./extension/plugin/instance")
        actuators = root.findall("./actuator/plugin")
        tendons = root.findall("./tendon/spatial")
        self.assertEqual(len(instances), 25)
        self.assertEqual(len(actuators), 25)
        self.assertEqual(len(tendons), 25)

        sites = {
            site.attrib["name"]: site for site in root.findall(".//site[@name]")
        }
        for digit in DIGITS:
            for cable in CABLES:
                name = f"{digit}_{cable}"
                instance = root.find(f"./extension/plugin/instance[@name='{name}_cable']")
                tendon = root.find(f"./tendon/spatial[@name='{name}_seed']")
                actuator = root.find(f"./actuator/plugin[@name='{name}_command']")
                self.assertIsNotNone(instance, name)
                self.assertIsNotNone(tendon, name)
                self.assertIsNotNone(actuator, name)
                self.assertEqual(actuator.attrib["gear"], "0")
                config = {
                    item.attrib["key"]: item.attrib["value"]
                    for item in instance.findall("config")
                }
                self.assertEqual(config["route_mode"], "surface")
                route_sites = [item.attrib["site"] for item in tendon.findall("site")]
                roles = [int(float(sites[item].attrib.get("user", "0"))) for item in route_sites]
                self.assertEqual(roles[0], 1)
                self.assertEqual(roles[-1], 1)
                self.assertEqual(roles.count(2), len(config["wrap_geoms"].split()))

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_model_loads_with_expected_dofs_and_routes(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.assertEqual(model.nq, 20)
        self.assertEqual(model.nv, 20)
        self.assertEqual(model.njnt, 20)
        self.assertEqual(model.nu, 25)
        self.assertEqual(model.ntendon, 25)
        self.assertEqual(model.nplugin, 25)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertTrue(np.isfinite(data.sensordata).all())
        for digit in DIGITS:
            for cable in CABLES:
                sensor_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_SENSOR, f"{digit}_{cable}_state"
                )
                address = model.sensor_adr[sensor_id]
                self.assertGreater(data.sensordata[address], 0.0)
                self.assertGreaterEqual(data.sensordata[address + 8], 1.0)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_index_flexors_generate_finite_joint_motion(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        commands = {
            "index_flexor_intermediate_command": 0.0035,
            "index_flexor_distal_command": 0.0060,
        }
        for name, value in commands.items():
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            data.ctrl[actuator_id] = value
        for _ in range(120):
            mujoco.mj_step(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertGreater(np.linalg.norm(data.qpos[:4]), 0.01)
        for name in (
            "index_flexor_intermediate_state",
            "index_flexor_distal_state",
        ):
            sensor_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, name
            )
            address = model.sensor_adr[sensor_id]
            self.assertGreaterEqual(data.sensordata[address + 8], 1.0)


class IndexCableDebugModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if PLUGIN.exists():
            mujoco.mj_loadPluginLibrary(str(PLUGIN.resolve()))

    def test_only_index_cable_system_remains(self):
        root = ET.parse(DEBUG_MODEL).getroot()
        instances = root.findall("./extension/plugin/instance")
        tendons = root.findall("./tendon/spatial")
        actuators = root.findall("./actuator/plugin")
        plugin_sensors = root.findall("./sensor/plugin")
        self.assertEqual(len(instances), 5)
        self.assertEqual(len(tendons), 5)
        self.assertEqual(len(actuators), 5)
        self.assertEqual(len(plugin_sensors), 5)
        for elements in (instances, tendons, actuators, plugin_sensors):
            self.assertTrue(
                all(item.attrib["name"].startswith("index_") for item in elements)
            )

        role_sites = root.findall(".//site[@user]")
        self.assertTrue(role_sites)
        self.assertTrue(
            all(site.attrib["name"].startswith("index_") for site in role_sites)
        )
        wrap_geoms = [
            geom.attrib["name"]
            for geom in root.findall(".//geom[@name]")
            if geom.attrib["name"].endswith("_wrap")
        ]
        self.assertTrue(wrap_geoms)
        self.assertTrue(all(name.startswith("index_") for name in wrap_geoms))
        self.assertIsNotNone(
            root.find("./worldbody/camera[@name='index_route_debug']")
        )

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_debug_model_loads_and_steps(self):
        model = mujoco.MjModel.from_xml_path(str(DEBUG_MODEL))
        self.assertEqual(model.nq, 20)
        self.assertEqual(model.nv, 20)
        self.assertEqual(model.njnt, 20)
        self.assertEqual(model.nu, 5)
        self.assertEqual(model.ntendon, 5)
        self.assertEqual(model.nplugin, 5)
        data = mujoco.MjData(model)
        distal_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "index_flexor_distal_command",
        )
        data.ctrl[distal_id] = 0.006
        for _ in range(120):
            mujoco.mj_step(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.sensordata).all())
        self.assertGreater(np.linalg.norm(data.qpos[:4]), 0.01)


class IndexMeshThreadingModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if PLUGIN.exists():
            mujoco.mj_loadPluginLibrary(str(PLUGIN.resolve()))

    def test_repaired_route_mesh_assets_are_declared_valid(self):
        manifest_path = ASSET_DIR / "index_route_mesh_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["voxel_pitch_mm"], 0.5)
        self.assertNotIn(str(Path.home()), manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(len(manifest["assets"]), 3)
        for record in manifest["assets"].values():
            self.assertTrue(record["watertight"])
            self.assertTrue(record["winding_consistent"])
            self.assertTrue(record["positive_volume"])
            self.assertGreater(record["route_faces"], 300)
            self.assertTrue((ASSET_DIR / record["output"]).is_file())

    def test_actual_mesh_routes_and_passive_ligaments_are_explicit(self):
        root = ET.parse(MESH_MODEL).getroot()
        self.assertFalse(
            any(
                geom.attrib.get("name", "").endswith("_wrap")
                for geom in root.findall(".//geom")
            )
        )
        surface_geoms = {
            geom.attrib["name"]
            for geom in root.findall(".//geom[@name]")
            if geom.attrib["name"].endswith("_surface")
        }
        self.assertEqual(
            surface_geoms,
            {
                "index_proximal_surface",
                "index_intermediate_surface",
                "index_distal_surface",
            },
        )

        active_instances = [
            instance
            for instance in root.findall("./extension/plugin/instance")
            if instance.find("config[@key='ctrl_mode']") is not None
        ]
        self.assertEqual(len(active_instances), 5)
        for instance in active_instances:
            config = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }
            self.assertEqual(config["mesh_route_mode"], "guided_surface")
            self.assertTrue(
                set(config["wrap_geoms"].split()).issubset(surface_geoms)
            )
            tendon = root.find(
                f"./tendon/spatial[@name='{config['route_tendon']}']"
            )
            self.assertIsNotNone(tendon)
            route_names = [site.attrib["site"] for site in tendon.findall("site")]
            self.assertFalse(any(name.endswith("_guide") for name in route_names))

        passive = [
            instance
            for instance in root.findall("./extension/plugin/instance")
            if instance.attrib["name"].startswith("index_ligament_")
        ]
        self.assertEqual(len(passive), 6)
        actuator_instances = {
            actuator.attrib["instance"]
            for actuator in root.findall("./actuator/plugin")
        }
        self.assertTrue(
            all(instance.attrib["name"] not in actuator_instances for instance in passive)
        )

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_mesh_routes_remain_valid_during_index_flexion(self):
        model = mujoco.MjModel.from_xml_path(str(MESH_MODEL))
        self.assertEqual(model.nq, 20)
        self.assertEqual(model.nu, 5)
        self.assertEqual(model.nplugin, 11)
        data = mujoco.MjData(model)
        for name, value in (
            ("index_flexor_intermediate_command", 0.0035),
            ("index_flexor_distal_command", 0.006),
        ):
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            data.ctrl[actuator_id] = value

        sensor_names = [
            f"index_{cable}_state" for cable in CABLES
        ] + [
            f"index_ligament_{joint}_{side}_state"
            for joint in ("mcp", "pip", "dip")
            for side in ("left", "right")
        ]
        sensor_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            for name in sensor_names
        ]
        for _ in range(240):
            mujoco.mj_step(model, data)
            for sensor_id in sensor_ids:
                address = model.sensor_adr[sensor_id]
                self.assertLess(data.sensordata[address + 8], 2.0)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertGreater(np.linalg.norm(data.qpos[:4]), 0.02)


if __name__ == "__main__":
    unittest.main()
