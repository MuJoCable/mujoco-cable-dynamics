import os
from pathlib import Path
import unittest

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


COMPOSITE_CYLINDER_XML = r"""
<mujoco model="composite_tangent_cylinders">
  <size nuser_site="1"/>
  <option gravity="0 0 0" timestep="0.001"/>
  <extension>
    <plugin plugin="mujoco.cable.unilateral">
      <instance name="cable">
        <config key="route_mode" value="surface"/>
        <config key="route_tendon" value="seed"/>
        <config key="wrap_geoms" value="left_surface right_surface"/>
        <config key="composite_merge_distance" value="0.001"/>
        <config key="route_hysteresis" value="0.0001"/>
        <config key="home_length" value="auto_initial"/>
        <config key="stiffness" value="100"/>
        <config key="visual_width" value="2"/>
      </instance>
    </plugin>
  </extension>
  <worldbody>
    <geom name="left_surface" type="cylinder" pos="-0.01 0 0"
          size="0.01 0.02" contype="0" conaffinity="0"/>
    <site name="start" pos="-0.04 -0.02 0" user="1"/>
    <site name="left_hint" pos="-0.01 0.01 0" user="2"/>
    <body name="right" pos="0.01 0 0">
      <joint name="surface_gap" type="slide" axis="1 0 0"
             range="0 0.002" damping="1"/>
      <geom name="right_surface" type="cylinder" pos="0 0 0"
            size="0.01 0.02" contype="0" conaffinity="0" mass="0.1"/>
      <site name="right_hint" pos="0 0.01 0" user="2"/>
      <site name="end" pos="0.03 -0.02 0" user="1"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="seed" width="0.000000001">
      <site site="start"/>
      <site site="left_hint"/>
      <site site="right_hint"/>
      <site site="end"/>
    </spatial>
  </tendon>
  <sensor><plugin name="state" instance="cable"/></sensor>
</mujoco>
"""


@unittest.skipUnless(PLUGIN.exists(), "compiled cable plugin is unavailable")
class CompositeSurfaceRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        mujoco.mj_loadPluginLibrary(str(PLUGIN.resolve()))

    def test_tangent_surfaces_merge_and_split_without_route_failure(self):
        model = mujoco.MjModel.from_xml_string(COMPOSITE_CYLINDER_XML)
        data = mujoco.MjData(model)
        address = int(model.sensor_adr[0])
        samples = np.concatenate(
            (
                np.linspace(0.0, 0.002, 81),
                np.linspace(0.002, 0.0, 81)[1:],
            )
        )
        lengths = []
        statuses = []
        for gap in samples:
            data.qpos[0] = gap
            data.qvel[0] = 0
            mujoco.mj_forward(model, data)
            lengths.append(float(data.sensordata[address]))
            statuses.append(int(round(data.sensordata[address + 8])))
        self.assertTrue(np.all(np.isfinite(lengths)))
        self.assertLessEqual(max(statuses), 1)
        self.assertLess(np.max(np.abs(np.diff(lengths))), 0.0001)
        self.assertLess(abs(lengths[0] - lengths[-1]), 1e-8)


if __name__ == "__main__":
    unittest.main()
