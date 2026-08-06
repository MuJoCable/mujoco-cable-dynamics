import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from scripts.check_faive_pip_mesh_contact import analyze as analyze_mesh_contact
from scripts.render_cpp_demo_screenshots import surface_line_segments


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "cable_plugin_demos/24_faive_index_pip_virtual_hinge_baseline.xml"
SURFACE = ROOT / "cable_plugin_demos/25_faive_index_pip_surface_cable.xml"
PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


def read_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices = []
    faces = []
    for line in path.read_text(encoding="ascii").splitlines():
        if line.startswith("v "):
            vertices.append([float(value) for value in line.split()[1:4]])
        elif line.startswith("f "):
            faces.append([int(value.split("/")[0]) - 1 for value in line.split()[1:4]])
    return np.asarray(vertices), np.asarray(faces, dtype=np.int64)


def edge_incidence(faces: np.ndarray) -> np.ndarray:
    edges = np.sort(
        np.concatenate(
            (faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]), axis=0
        ),
        axis=1,
    )
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return counts


class FaiveIndexPipComparisonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if PLUGIN.exists():
            mujoco.mj_loadPluginLibrary(str(PLUGIN.resolve()))

    def test_reference_assets_include_license(self):
        asset_dir = ROOT / "cable_plugin_demos/assets/faive_index_pip"
        self.assertTrue((asset_dir / "index_pp.stl").is_file())
        self.assertTrue((asset_dir / "index_mp.stl").is_file())
        license_text = (asset_dir / "LICENSE.faive-apache-2.0.txt").read_text()
        self.assertIn("Apache License", license_text)
        self.assertIn("Version 2.0", license_text)

    def test_repaired_route_shells_are_closed_oriented_manifolds(self):
        asset_dir = ROOT / "cable_plugin_demos/assets/faive_index_pip"
        for filename in ("index_pp_outer.obj", "index_mp_outer.obj"):
            vertices, faces = read_obj(asset_dir / filename)
            self.assertGreater(len(faces), 100)
            self.assertTrue(np.all(edge_incidence(faces) == 2), filename)
            triangles = vertices[faces]
            edge_lengths = np.linalg.norm(
                np.concatenate(
                    (
                        triangles[:, 1] - triangles[:, 0],
                        triangles[:, 2] - triangles[:, 1],
                        triangles[:, 0] - triangles[:, 2],
                    ),
                    axis=0,
                ),
                axis=1,
            )
            self.assertLessEqual(edge_lengths.max(), 0.002000001, filename)
            doubled_area = np.linalg.norm(
                np.cross(
                    triangles[:, 1] - triangles[:, 0],
                    triangles[:, 2] - triangles[:, 0],
                ),
                axis=1,
            )
            self.assertTrue(np.all(doubled_area > 1e-14), filename)
            signed_volume = np.einsum(
                "ij,ij->i",
                triangles[:, 0],
                np.cross(triangles[:, 1], triangles[:, 2]),
            ).sum() / 6.0
            self.assertGreater(signed_volume, 0, filename)

    def test_blue_control_cable_uses_requested_centerline_span(self):
        root = ET.parse(SURFACE).getroot()
        sites = {site.attrib["name"]: site for site in root.findall(".//site[@name]")}
        proximal = np.fromstring(sites["extensor_prox"].attrib["pos"], sep=" ")
        distal = np.fromstring(sites["extensor_dist"].attrib["pos"], sep=" ")
        np.testing.assert_allclose(
            proximal, [-0.034025209, 0.151656207, 0.026339542], atol=1e-12
        )
        np.testing.assert_allclose(
            distal, [-0.037926041, 0.185185586, 0.030409466], atol=1e-12
        )

    def test_baseline_preserves_two_virtual_hinges(self):
        model = mujoco.MjModel.from_xml_path(str(BASELINE))
        self.assertEqual(model.nq, 2)
        self.assertEqual(model.nv, 2)
        self.assertEqual(model.nu, 1)
        self.assertEqual(model.ntendon, 1)
        expected_axis = np.array(
            [-0.990425994580321, -0.104241812967506, -0.090498583905110]
        )
        np.testing.assert_allclose(model.jnt_axis[0], expected_axis, atol=1e-9)
        np.testing.assert_allclose(model.jnt_axis[1], expected_axis, atol=1e-9)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_surface_model_has_no_hidden_kinematic_joint(self):
        model = mujoco.MjModel.from_xml_path(str(SURFACE))
        self.assertEqual(model.nq, 7)
        self.assertEqual(model.nv, 6)
        self.assertEqual(model.nu, 2)
        self.assertEqual(model.neq, 0)
        self.assertEqual(model.njnt, 1)
        self.assertEqual(model.jnt_type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(model.nflex, 2)
        for geom_name in ("proximal_route_surface", "distal_route_surface"):
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.assertGreaterEqual(geom, 0)
            self.assertEqual(model.geom_type[geom], mujoco.mjtGeom.mjGEOM_MESH)
            self.assertEqual(model.geom_contype[geom], 0)
            self.assertEqual(model.geom_conaffinity[geom], 0)

        root = ET.parse(SURFACE).getroot()
        surface_instances = [
            instance
            for instance in root.findall("./extension/plugin/instance")
            if any(
                config.attrib.get("key") == "route_mode"
                and config.attrib.get("value") == "surface"
                for config in instance.findall("config")
            )
        ]
        self.assertEqual(len(surface_instances), 4)
        for instance in surface_instances:
            config = {
                item.attrib["key"]: item.attrib["value"]
                for item in instance.findall("config")
            }
            self.assertEqual(config["mesh_route_mode"], "guided_surface")
            np.testing.assert_allclose(
                np.fromstring(config["mesh_guide_axis"], sep=" "),
                [-0.990425995, -0.104241813, -0.090498584],
                atol=1e-9,
            )
            self.assertEqual(float(config["mesh_guide_weight"]), 20.0)
            self.assertEqual(float(config["composite_merge_distance"]), 0.00080)
            expected_wraps = "proximal_route_surface distal_route_surface"
            self.assertEqual(" ".join(config["wrap_geoms"].split()), expected_wraps)

        for tendon_name in (
            "pip_flexor_seed",
            "pip_extensor_seed",
            "ligament_right_upper_seed",
            "ligament_right_lower_seed",
        ):
            tendon = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name
            )
            self.assertLessEqual(model.tendon_width[tendon], 1e-8, tendon_name)

        for actuator_name in ("pip_flexor_command", "pip_extensor_command"):
            actuator = root.find(f".//actuator/plugin[@name='{actuator_name}']")
            self.assertIsNotNone(actuator)
            self.assertEqual(float(actuator.attrib["gear"]), 0.0)

        expected_active_routes = {
            "pip_flexor_seed": [
                "flexor_prox",
                "flexor_prox_hint",
                "flexor_dist_hint",
                "flexor_dist",
            ],
            "pip_extensor_seed": [
                "extensor_prox",
                "extensor_prox_hint",
                "extensor_dist_hint",
                "extensor_dist",
            ],
        }
        for tendon_name, expected_sites in expected_active_routes.items():
            tendon = root.find(f".//tendon/spatial[@name='{tendon_name}']")
            self.assertIsNotNone(tendon)
            self.assertEqual(
                [site.attrib["site"] for site in tendon.findall("site")],
                expected_sites,
            )

        right_upper_seed = root.find(
            ".//tendon/spatial[@name='ligament_right_upper_seed']"
        )
        self.assertIsNotNone(right_upper_seed)
        self.assertEqual(
            [site.attrib["site"] for site in right_upper_seed.findall("site")],
            [
                "ligament_right_upper_prox",
                "ligament_right_upper_prox_bridge_hint",
                "ligament_right_upper_dist_bridge_hint",
                "ligament_right_upper_dist",
            ],
        )
        sites = {site.attrib["name"]: site for site in root.findall(".//site[@name]")}
        self.assertEqual(sites["ligament_right_upper_prox_hint"].attrib["user"], "0")
        self.assertEqual(sites["ligament_right_upper_dist_hint"].attrib["user"], "0")
        for name in (
            "ligament_right_upper_prox",
            "ligament_right_lower_prox",
        ):
            self.assertAlmostEqual(
                np.fromstring(sites[name].attrib["pos"], sep=" ")[0],
                -0.032529261,
                places=9,
            )
        for name in (
            "ligament_right_upper_dist",
            "ligament_right_lower_dist",
        ):
            self.assertAlmostEqual(
                np.fromstring(sites[name].attrib["pos"], sep=" ")[0],
                -0.036771730,
                places=9,
            )
        self.assertFalse(any("ligament_left" in name for name in sites))

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_guided_surface_cables_remain_wrapped_and_ignore_runtime_hints(self):
        model = mujoco.MjModel.from_xml_path(str(SURFACE))
        data = mujoco.MjData(model)
        for _ in range(20):
            mujoco.mj_forward(model, data)

        route_names = (
            "pip_flexor_seed",
            "pip_extensor_seed",
            "ligament_right_upper_seed",
            "ligament_right_lower_seed",
        )

        def route_points():
            grouped = {}
            for tendon_id, start, end, _ in surface_line_segments(model, data):
                name = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id
                )
                grouped.setdefault(name, []).append((start.copy(), end.copy()))
            return {
                name: np.asarray(
                    [grouped[name][0][0], *(segment[1] for segment in grouped[name])]
                )
                for name in route_names
            }

        initial_points = route_points()
        initial_lengths = data.sensordata[
            [int(model.sensor_adr[index]) for index in range(4)]
        ].copy()
        for name, points in initial_points.items():
            self.assertGreaterEqual(len(points) - 1, 6, name)
            chord = points[-1] - points[0]
            chord_length = np.linalg.norm(chord)
            deviation = np.linalg.norm(
                np.cross(points - points[0], chord), axis=1
            ).max() / chord_length
            self.assertGreater(deviation, 0.001, name)
        right_upper = initial_points["ligament_right_upper_seed"]
        self.assertGreaterEqual(len(right_upper) - 1, 30)
        self.assertGreater(np.ptp(right_upper[1:-1, 2]), 0.003)

        role_hints = np.flatnonzero(np.isclose(model.site_user[:, 0], 2.0))
        model.site_pos[role_hints] += np.array([0.012, -0.008, 0.006])
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(
            data.sensordata[
                [int(model.sensor_adr[index]) for index in range(4)]
            ],
            initial_lengths,
            atol=1e-10,
        )
        final_points = route_points()
        for name in route_names:
            np.testing.assert_allclose(final_points[name], initial_points[name], atol=1e-10)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_surface_model_neutral_rollout_is_stable(self):
        model = mujoco.MjModel.from_xml_path(str(SURFACE))
        data = mujoco.MjData(model)
        tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "distal_tip")
        mujoco.mj_forward(model, data)
        initial_tip = data.site_xpos[tip].copy()
        contact_steps = 0
        invalid_steps = 0
        maximum_contacts = 0
        steps = round(0.25 / model.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(model, data)
            contact_steps += data.ncon > 0
            maximum_contacts = max(maximum_contacts, data.ncon)
            for sensor in range(model.nsensor):
                if model.sensor_dim[sensor] < 12:
                    continue
                address = model.sensor_adr[sensor]
                invalid_steps += data.sensordata[address + 8] >= 2
        self.assertTrue(np.all(np.isfinite(data.qpos)))
        self.assertGreater(contact_steps / steps, 0.98)
        self.assertEqual(invalid_steps, 0)
        self.assertLessEqual(maximum_contacts, 256)
        # The 0.1 mm predictive contact margin produces a small initialization
        # settling motion while keeping the actual outer shells disjoint.
        self.assertLess(np.linalg.norm(data.site_xpos[tip] - initial_tip), 0.00025)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_surface_flexor_actuation_keeps_every_route_valid(self):
        model = mujoco.MjModel.from_xml_path(str(SURFACE))
        data = mujoco.MjData(model)
        flexor = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "pip_flexor_command"
        )
        steps = round(0.5 / model.opt.timestep)
        maximum_status = 0
        for _ in range(steps):
            phase = float(np.clip((data.time - 0.15) / 0.75, 0.0, 1.0))
            data.ctrl[flexor] = 0.003 * phase * phase * (3.0 - 2.0 * phase)
            mujoco.mj_step(model, data)
            for sensor in range(model.nsensor):
                if model.sensor_dim[sensor] < 12:
                    continue
                address = model.sensor_adr[sensor]
                maximum_status = max(
                    maximum_status,
                    int(round(data.sensordata[address + 8])),
                )
        self.assertTrue(np.all(np.isfinite(data.qpos)))
        self.assertEqual(maximum_status, 0)
        self.assertGreater(data.ncon, 0)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_dual_actuation_has_no_shell_or_cable_penetration(self):
        report = analyze_mesh_contact(
            PLUGIN.resolve(),
            SURFACE.resolve(),
            steps=2500,
            flexor=0.003,
            extensor=0.0027,
            route_samples=17,
            shell_samples=3,
        )
        summary = report["summary"]
        self.assertEqual(summary["maximum_shell_intersections"], 0)
        self.assertEqual(summary["maximum_visual_mesh_intersections"], 0)
        self.assertEqual(summary["inside_endpoint_count"], 0)
        self.assertEqual(summary["penetrating_route_segment_count"], 0)
        self.assertEqual(summary["maximum_route_status"], 0)
        self.assertLessEqual(summary["maximum_contacts"], 256)
        interface_routes = [
            route
            for snapshot in report["snapshots"]
            for route in snapshot["interface_routes"]
        ]
        # The two surface envelopes now meet through a finite, collision-free
        # common-tangent bridge instead of two seed-face projections separated
        # by the physical shell gap.
        self.assertGreater(
            min(route["interface_gap_mm"] for route in interface_routes), 0.5
        )
        self.assertLess(summary["maximum_interface_gap_mm"], 10.0)
        # The current faceted Faive shells produce a deterministic 21.2 deg
        # worst-case bridge angle. Keep the regression bound close enough to
        # catch topology changes while documenting the discretization limit.
        self.assertLess(summary["maximum_interface_tangent_angle_deg"], 22.0)
        self.assertLess(summary["maximum_interface_surface_residual_mm"], 1e-3)

    @unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
    def test_invalid_surface_route_does_not_draw_stale_world_path(self):
        model = mujoco.MjModel.from_xml_path(str(SURFACE))
        data = mujoco.MjData(model)
        option = mujoco.MjvOption()
        perturb = mujoco.MjvPerturb()
        camera = mujoco.MjvCamera()
        scene = mujoco.MjvScene(model, maxgeom=2000)

        mujoco.mj_forward(model, data)
        mujoco.mjv_updateScene(
            model,
            data,
            option,
            perturb,
            camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            scene,
        )
        valid_lines = sum(
            int(scene.geoms[index].objtype)
            == int(mujoco.mjtObj.mjOBJ_UNKNOWN)
            for index in range(scene.ngeom)
        )
        self.assertGreater(valid_lines, 0)

        data.qpos[1] -= 0.015
        mujoco.mj_forward(model, data)
        route_tendons = [
            "ligament_right_upper_seed",
            "ligament_right_lower_seed",
        ]
        invalid_tendon_ids = set()
        for sensor, tendon_name in zip(range(2, 4), route_tendons, strict=True):
            status = int(
                round(data.sensordata[int(model.sensor_adr[sensor]) + 8])
            )
            if status >= 2:
                invalid_tendon_ids.add(
                    mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name
                    )
                )
        self.assertTrue(invalid_tendon_ids)

        mujoco.mjv_updateScene(
            model,
            data,
            option,
            perturb,
            camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            scene,
        )
        stale_invalid_lines = sum(
            int(scene.geoms[index].objtype)
            == int(mujoco.mjtObj.mjOBJ_UNKNOWN)
            and int(scene.geoms[index].objid) in invalid_tendon_ids
            for index in range(scene.ngeom)
        )
        self.assertEqual(stale_invalid_lines, 0)


if __name__ == "__main__":
    unittest.main()
