from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET

import mujoco

from scripts.analyze_cpho_2018_problem3 import (
    load_parameters,
    numeric,
    official_coefficients,
    official_transition_time,
    simulate,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "cable_plugin_demos/30_cpho_2018_problem3_massive_rope.xml"


class Cpho2018Problem3Test(unittest.TestCase):
    def test_model_is_an_explicit_massive_rope_boundary_benchmark(self):
        root = ET.parse(MODEL).getroot()
        self.assertIsNone(root.find("./extension"))
        self.assertIsNotNone(
            root.find("./worldbody/body/joint[@name='rope_transport']")
        )
        tendon = root.find("./tendon/spatial[@name='massive_rope_visual']")
        self.assertIsNotNone(tendon)
        self.assertIsNotNone(tendon.find("./geom[@geom='drive_pulley']"))

    def test_official_maximum_speed_has_both_branches(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        parameters = load_parameters(model)
        coefficients = official_coefficients(parameters)
        self.assertEqual(int(data.ten_wrapnum[0]), 4)
        expected_visual_length = (
            2.0 * (parameters.height - 0.012)
            + math.pi * parameters.radius
        )
        self.assertAlmostEqual(
            float(data.ten_length[0]), expected_visual_length, places=10
        )
        expected_squared = (
            parameters.height
            * parameters.gravity
            * (1.0 - math.exp(-parameters.friction * math.pi))
            + parameters.radius
            * parameters.gravity
            * (2.0 * parameters.friction / (1.0 + parameters.friction**2))
            * (1.0 + math.exp(-parameters.friction * math.pi))
        )
        self.assertAlmostEqual(
            coefficients.sliding_speed_limit**2, expected_squared, places=12
        )

        sliding_omega = numeric(model, "sliding_case_omega")
        sticking_omega = numeric(model, "sticking_case_omega")
        self.assertIsNone(
            official_transition_time(
                parameters.radius * sliding_omega, coefficients
            )
        )
        self.assertIsNotNone(
            official_transition_time(
                parameters.radius * sticking_omega, coefficients
            )
        )

    def test_mujoco_transport_matches_official_solution(self):
        _, summary = simulate(
            MODEL,
            sliding_duration=3.0,
            sticking_duration=1.2,
            sample_stride=25,
        )
        self.assertTrue(summary["pass"], summary)
        cases = {case["case"]: case for case in summary["cases"]}
        self.assertEqual(
            cases["sliding_limited"]["predicted_branch"], "sliding_limited"
        )
        self.assertEqual(
            cases["stick_limited"]["predicted_branch"], "stick_limited"
        )


if __name__ == "__main__":
    unittest.main()
