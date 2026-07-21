import math
import unittest

from mujoco_cable.capstan import capstan_limit_ratio, impending_slip_tensions, within_no_slip_range


class CapstanLawTest(unittest.TestCase):
    def test_impending_slip_matches_euler_eytelwein(self):
        result = impending_slip_tensions(mu=0.3, theta=math.pi, tension_low=10.0)
        self.assertAlmostEqual(result.tension_ratio, math.exp(0.3 * math.pi))
        self.assertAlmostEqual(result.tension_high, 10.0 * math.exp(0.3 * math.pi))
        self.assertEqual(result.slip_state, "impending_slip_high_side")

    def test_frictionless_case_has_equal_tension(self):
        result = impending_slip_tensions(mu=0.0, theta=2.0 * math.pi, tension_low=12.0)
        self.assertAlmostEqual(result.tension_high, 12.0)
        self.assertAlmostEqual(result.tension_ratio, 1.0)
        self.assertEqual(result.slip_state, "frictionless")

    def test_no_slip_range_is_symmetric_in_log_tension_ratio(self):
        limit = capstan_limit_ratio(mu=0.2, theta=math.pi)
        self.assertTrue(within_no_slip_range(10.0, 10.0 * limit, 0.2, math.pi))
        self.assertTrue(within_no_slip_range(10.0, 10.0 / limit, 0.2, math.pi))
        self.assertFalse(within_no_slip_range(10.0, 10.0 * limit * 1.01, 0.2, math.pi))

if __name__ == "__main__":
    unittest.main()
