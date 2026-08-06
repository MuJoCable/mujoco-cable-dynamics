import importlib.util
import os
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


def load_script():
    path = ROOT / "scripts" / "check_rolling_joint_wrap_geometry.py"
    spec = importlib.util.spec_from_file_location("check_rolling_joint_wrap_geometry", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load script module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RollingJointWrapGeometryTest(unittest.TestCase):
    def test_demo13_wraps_each_cable_tangent_to_required_surfaces(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        module = load_script()
        report = module.analyze_model(plugin_path=DEFAULT_PLUGIN)
        self.assertTrue(report["pass"], report)
        self.assertLessEqual(report["max_surface_error"], 1e-7)
        self.assertLessEqual(report["max_tangent_error"], 1e-6)
        self.assertEqual(report["missing_required"], [])


if __name__ == "__main__":
    unittest.main()
