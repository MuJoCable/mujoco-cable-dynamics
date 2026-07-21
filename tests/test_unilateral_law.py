import unittest

from mujoco_cable import UnilateralCableLaw


class UnilateralCableLawTest(unittest.TestCase):
    def test_slack_never_pushes(self):
        law = UnilateralCableLaw(stiffness=100.0, damping=10.0, slack_threshold=0.01)
        result = law.evaluate(path_length=0.9, free_length=1.0, path_velocity=1.0)
        self.assertEqual(result.tension, 0.0)
        self.assertFalse(result.taut)

    def test_taut_matches_spring_damper(self):
        law = UnilateralCableLaw(stiffness=100.0, damping=10.0, max_tension=1000.0)
        result = law.evaluate(path_length=1.1, free_length=1.0, path_velocity=0.2)
        self.assertLess(abs(result.tension - 12.0), 1e-12)
        self.assertTrue(result.taut)

    def test_saturation(self):
        law = UnilateralCableLaw(stiffness=1000.0, max_tension=5.0)
        result = law.evaluate(path_length=1.1, free_length=1.0, path_velocity=0.0)
        self.assertEqual(result.tension, 5.0)
        self.assertTrue(result.saturated)


if __name__ == "__main__":
    unittest.main()
