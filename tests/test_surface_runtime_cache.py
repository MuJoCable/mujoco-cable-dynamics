"""Small, asset-free regressions for cached routes and newer cable controls."""

import copy
import os
from pathlib import Path
import unittest

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
LIBRARY = Path(os.environ.get(
    "CABLE_PLUGIN_LIBRARY", str(ROOT / "build/plugin/libcable_unilateral.dylib")
))

MESH_XML = """<mujoco><size nuser_site="1"/>
<option gravity="0 0 0"/>
<extension><plugin plugin="mujoco.cable.unilateral"><instance name="rope">
<config key="route_mode" value="surface"/>
<config key="route_tendon" value="seed"/>
<config key="wrap_geoms" value="pulley"/>
<config key="home_length" value="auto_initial"/>
<config key="stiffness" value="800"/>
<config key="pretension_offset" value=".001"/>
</instance></plugin></extension>
<asset><mesh name="shape" vertex="-.02 -.02 -.05 .02 -.02 -.05 .02 .02 -.05 -.02 .02 -.05 -.02 -.02 .05 .02 -.02 .05 .02 .02 .05 -.02 .02 .05"/></asset>
<worldbody><body name="assembly"><freejoint/>
<geom name="pulley" type="mesh" mesh="shape" mass="1"/>
<site name="a" pos="-.04 0 0" user="1"/>
<site name="h" pos="0 .02 0" user="2"/>
<site name="b" pos=".04 0 0" user="1"/>
</body></worldbody>
<tendon><spatial name="seed"><site site="a"/><site site="h"/><site site="b"/></spatial></tendon>
<sensor><plugin instance="rope"/></sensor></mujoco>"""


def cylinder_xml(extra=""):
    return f"""<mujoco><size nuser_site="1"/><option gravity="0 0 0"/>
<extension><plugin plugin="mujoco.cable.unilateral"><instance name="rope">
<config key="route_mode" value="surface"/>
<config key="route_tendon" value="seed"/>
<config key="wrap_geoms" value="pulley"/>
<config key="home_length" value="auto_initial"/>
<config key="stiffness" value="800"/>
<config key="pretension_offset" value=".001"/>{extra}
</instance></plugin></extension>
<worldbody><geom name="pulley" type="cylinder" size=".02 .05"/>
<site name="a" pos="-.04 0 0" user="1"/>
<site name="h" pos="0 .02 0" user="2"/>
<site name="b" pos=".04 0 0" user="1"/></worldbody>
<tendon><spatial name="seed"><site site="a"/><site site="h"/><site site="b"/></spatial></tendon>
<sensor><plugin instance="rope"/></sensor></mujoco>"""


class SurfaceRuntimeCacheTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not LIBRARY.is_file():
            raise unittest.SkipTest("Build the plugin or set CABLE_PLUGIN_LIBRARY")
        mujoco.mj_loadPluginLibrary(str(LIBRARY))

    def model(self, xml=MESH_XML):
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        for _ in range(8):
            mujoco.mj_forward(model, data)
        return model, data

    def test_rigid_transform_preserves_length_and_internal_wrench(self):
        model, data = self.model()
        length = data.sensordata[0]
        for angle in (0.0, 0.4, 1.2):
            data.qpos[:3] = [0.1, -0.2, 0.3]
            mujoco.mju_axisAngle2Quat(data.qpos[3:], np.array([0., 1., 0.]), angle)
            data.qvel[:] = [0.1, -0.2, 0.3, 0.4, 0.5, -0.6]
            mujoco.mj_forward(model, data)
            self.assertLessEqual(data.sensordata[8], 1)
            self.assertAlmostEqual(data.sensordata[0], length, places=10)
            self.assertAlmostEqual(data.sensordata[1], 0., places=10)
            self.assertGreater(data.sensordata[5], 0.)
            np.testing.assert_allclose(data.qfrc_passive, 0., atol=1e-10)

    def test_moved_hard_anchor_invalidates_cache(self):
        model, data = self.model()
        length = data.sensordata[0]
        model.site_pos[model.site("a").id, 0] -= .001
        mujoco.mj_forward(model, data)
        self.assertGreater(data.sensordata[0] - length, .0005)
        self.assertLessEqual(data.sensordata[8], 1)

    def test_runtime_hint_is_not_a_hard_anchor(self):
        model, data = self.model()
        length = data.sensordata[0]
        model.site_pos[model.site("h").id] += [0.1, -0.2, 0.3]
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.sensordata[0], length, places=10)

    def test_copied_data_has_independent_runtime(self):
        model, data = self.model()
        other = copy.copy(data)
        initial = data.sensordata.copy()
        other.qpos[:3] = [0.5, 0.6, 0.7]
        mujoco.mj_forward(model, other)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.sensordata[:11], initial[:11], atol=1e-10)
        self.assertFalse(np.shares_memory(data.qpos, other.qpos))

    def test_reset_reinitializes_local_solution(self):
        model, data = self.model()
        length = data.sensordata[0]
        data.qpos[:3] = [0.1, 0.2, 0.3]
        mujoco.mj_forward(model, data)
        mujoco.mj_resetData(model, data)
        for _ in range(8):
            mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.sensordata[0], length, places=10)
        self.assertEqual(data.sensordata[8], 0.)

    def test_body_wrench_matches_length_gradient(self):
        xml = MESH_XML.replace(
            '<site name="b" pos=".04 0 0" user="1"/>',
            '<body name="anchor" pos=".04 0 0"><joint name="slide" type="slide" axis="1 0 0"/>'
            '<geom type="sphere" size=".001" mass=".01" contype="0" conaffinity="0"/>'
            '<site name="b" user="1"/></body>',
        )
        model, data = self.model(xml)
        tension = float(data.sensordata[5])
        force = float(data.qfrc_passive[model.joint("slide").dofadr[0]])
        qadr = model.joint("slide").qposadr[0]
        lengths = []
        for offset in (-1e-7, 1e-7):
            data.qpos[qadr] = offset
            for _ in range(8):
                mujoco.mj_forward(model, data)
            lengths.append(float(data.sensordata[0]))
        derivative = (lengths[1] - lengths[0]) / 2e-7
        self.assertGreater(tension, 0.)
        self.assertAlmostEqual(force, -tension * derivative, delta=1e-5)

    def test_shared_control_source_and_scale(self):
        xml = cylinder_xml(
            '<config key="control_source_actuator" value="command"/>'
            '<config key="control_scale" value=".5"/>'
        ).replace(
            '<instance name="rope">',
            '<instance name="driver"><config key="home_length" value=".1"/></instance>'
            '<instance name="rope">',
        ).replace(
            '<sensor>',
            '<actuator><plugin name="command" instance="driver" tendon="seed" gear="0"/></actuator><sensor>',
        )
        model, data = self.model(xml)
        data.ctrl[0] = .004
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.sensordata[3], .002, places=12)

    def test_actuated_velocity_friction_can_step(self):
        xml = cylinder_xml(
            '<config key="capstan_direction" value="velocity"/>'
            '<config key="capstan_mu" value=".15"/>'
        ).replace(
            '<sensor>',
            '<actuator><plugin name="command" instance="rope" tendon="seed" gear="0"/></actuator><sensor>',
        )
        model, data = self.model(xml)
        for value in (.0001, .0002, .0003, .0002):
            data.ctrl[0] = value
            mujoco.mj_step(model, data)
            self.assertEqual(data.sensordata[8], 0.)
            self.assertTrue(np.all(np.isfinite(data.sensordata)))

    def test_cylinder_extra_turn_increases_length(self):
        lengths = []
        for turns in (0, 1):
            xml = cylinder_xml(
                '<config key="cylinder_winding_direction" value="negative"/>'
                f'<config key="cylinder_extra_turns" value="{turns}"/>'
            )
            _, data = self.model(xml)
            self.assertEqual(data.sensordata[8], 0.)
            lengths.append(data.sensordata[0])
        # Runtime length uses a sampled polyline rather than the exact arc.
        self.assertAlmostEqual(lengths[1] - lengths[0], 2 * np.pi * .02, delta=2e-4)

    def test_invalid_cylinder_options_fail_initialization(self):
        options = {
            "cylinder_winding_direction": "sideways",
            "cylinder_extra_turns": "0.5",
            "cylinder_min_wrap_angle": "7",
        }
        for key, value in options.items():
            with self.subTest(key=key), self.assertRaises(ValueError):
                mujoco.MjModel.from_xml_string(cylinder_xml(
                    f'<config key="{key}" value="{value}"/>'
                ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
