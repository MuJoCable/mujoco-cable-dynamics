"""Smoke-test the standalone MJCF downloads using native MuJoCo only."""

from pathlib import Path
import unittest

import mujoco
import numpy as np


MODELS = Path(__file__).resolve().parents[1] / "_models"


class NativeReferenceTest(unittest.TestCase):
    def load(self, name):
        model = mujoco.MjModel.from_xml_path(str(MODELS / name))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertEqual(model.nplugin, 0)
        return model, data

    def test_positive_pull_shortens_the_wrapped_route(self):
        model, data = self.load("mujoco_native_cable.xml")
        start = data.sensor("cable_length").data[0]
        straight = np.linalg.norm(data.site("fixed_end").xpos - data.site("moving_end").xpos)
        self.assertGreater(start, straight)
        data.actuator("pull").ctrl[0] = 0.75
        for _ in range(20):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.sensor("pull_output").data[0], 0.75)
        self.assertLess(data.sensor("cable_length").data[0], start)
        self.assertLess(data.sensor("cable_rate").data[0], 0)

    def test_four_bar_remains_closed_while_actuated(self):
        model, data = self.load("mujoco_four_bar.xml")
        self.assertEqual((model.nv, model.neq), (3, 1))

        def closure_error():
            return np.linalg.norm(data.site("coupler_tip").xpos - data.site("rocker_tip").xpos)

        self.assertLess(closure_error(), 1e-10)
        data.actuator("crank_target").ctrl[0] = 0.30
        for _ in range(1000):
            mujoco.mj_step(model, data)
            self.assertLess(closure_error(), 0.001)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.joint("crank_hinge").qpos[0], 0.30, places=3)


if __name__ == "__main__":
    unittest.main()
