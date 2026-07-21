import os
from pathlib import Path
import unittest

import mujoco


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


def dented_cube_xml(mesh_route_mode: str) -> str:
    vertices = (
        "-.05 -.05 -.05  .05 -.05 -.05  .05 .05 -.05  -.05 .05 -.05 "
        "-.05 -.05 .05   .05 -.05 .05   .05 .05 .05   -.05 .05 .05 "
        "0 0 0"
    )
    faces = (
        "0 2 1  0 3 2 "
        "0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7 "
        "4 5 8  5 6 8  6 7 8  7 4 8"
    )
    return f"""
<mujoco>
  <compiler autolimits="true"/>
  <size nuser_site="1"/>
  <option gravity="0 0 0"/>
  <extension>
    <plugin plugin="mujoco.cable.unilateral">
      <instance name="cable">
        <config key="route_mode" value="surface"/>
        <config key="mesh_route_mode" value="{mesh_route_mode}"/>
        <config key="route_tendon" value="seed"/>
        <config key="wrap_geoms" value="dent"/>
        <config key="home_length" value="auto_initial"/>
        <config key="stiffness" value="10"/>
      </instance>
    </plugin>
  </extension>
  <asset><mesh name="dent_mesh" vertex="{vertices}" face="{faces}"/></asset>
  <worldbody>
    <geom name="dent" type="mesh" mesh="dent_mesh" contype="0" conaffinity="0"/>
    <site name="start" pos="-.12 0 0" user="1"/>
    <site name="hint" pos="0 0 .055" user="2"/>
    <site name="end" pos=".12 0 0" user="1"/>
  </worldbody>
  <tendon>
    <spatial name="seed" width="0.000000001">
      <site site="start"/><site site="hint"/><site site="end"/>
    </spatial>
  </tendon>
  <sensor><plugin instance="cable"/></sensor>
</mujoco>
"""


def self_intersecting_tetrahedra_xml() -> str:
    vertices = (
        "0 0 0  .08 0 0  0 .08 0  0 0 .08 "
        ".02 .02 .02  .10 .02 .02  .02 .10 .02  .02 .02 .10"
    )
    faces = (
        "0 2 1  0 1 3  0 3 2  1 2 3 "
        "4 6 5  4 5 7  4 7 6  5 6 7"
    )
    return f"""
<mujoco>
  <compiler autolimits="true"/>
  <size nuser_site="1"/>
  <extension>
    <plugin plugin="mujoco.cable.unilateral">
      <instance name="cable">
        <config key="route_mode" value="surface"/>
        <config key="mesh_route_mode" value="taut_obstacle"/>
        <config key="route_tendon" value="seed"/>
        <config key="wrap_geoms" value="obstacle"/>
      </instance>
    </plugin>
  </extension>
  <asset><mesh name="intersecting" vertex="{vertices}" face="{faces}"/></asset>
  <worldbody>
    <geom name="obstacle" type="mesh" mesh="intersecting"
          contype="0" conaffinity="0"/>
    <site name="start" pos="-.12 0 .04" user="1"/>
    <site name="hint" pos=".04 .04 .11" user="2"/>
    <site name="end" pos=".14 0 .04" user="1"/>
  </worldbody>
  <tendon>
    <spatial name="seed" width="0.000000001">
      <site site="start"/><site site="hint"/><site site="end"/>
    </spatial>
  </tendon>
</mujoco>
"""


class NonconvexSurfaceRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if DEFAULT_PLUGIN.exists():
            mujoco.mj_loadPluginLibrary(str(DEFAULT_PLUGIN))

    def require_plugin(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")

    def test_default_convex_mode_rejects_dented_mesh(self):
        self.require_plugin()
        with self.assertRaises(ValueError):
            mujoco.MjModel.from_xml_string(dented_cube_xml("convex_surface"))

    def test_taut_obstacle_accepts_dented_mesh_and_ignores_runtime_hint(self):
        self.require_plugin()
        model = mujoco.MjModel.from_xml_string(dented_cube_xml("taut_obstacle"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertGreater(float(data.sensordata[0]), 0.24)
        self.assertEqual(int(round(float(data.sensordata[8]))), 0)
        self.assertLess(float(data.sensordata[9]), 1e-5)
        self.assertLess(float(data.sensordata[10]), 1e-7)

        for _ in range(20):
            mujoco.mj_forward(model, data)
        initial_length = float(data.sensordata[0])
        hint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hint")
        model.site_pos[hint_id] += (0.03, -0.04, 0.02)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(float(data.sensordata[0]), initial_length, places=10)

    def test_guided_surface_accepts_dented_mesh_and_ignores_runtime_hint(self):
        self.require_plugin()
        model = mujoco.MjModel.from_xml_string(dented_cube_xml("guided_surface"))
        data = mujoco.MjData(model)
        for _ in range(20):
            mujoco.mj_forward(model, data)
        initial_length = float(data.sensordata[0])
        self.assertGreater(initial_length, 0.24)
        self.assertEqual(int(round(float(data.sensordata[8]))), 0)

        hint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hint")
        model.site_pos[hint_id] += (0.03, -0.04, 0.02)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(float(data.sensordata[0]), initial_length, places=10)

    def test_taut_obstacle_rejects_self_intersecting_mesh(self):
        self.require_plugin()
        with self.assertRaisesRegex(ValueError, "self-intersecting"):
            mujoco.MjModel.from_xml_string(self_intersecting_tetrahedra_xml())


if __name__ == "__main__":
    unittest.main()
