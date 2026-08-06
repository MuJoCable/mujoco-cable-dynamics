import importlib.util
import os
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


def load_script():
    path = ROOT / "scripts" / "check_surface_pulley_demo.py"
    spec = importlib.util.spec_from_file_location("check_surface_pulley_demo", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load script module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SurfacePulleyDemoTest(unittest.TestCase):
    def test_demo15_lifts_free_payload_over_runtime_surface_envelope(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        module = load_script()
        report = module.analyze_demo(plugin_path=DEFAULT_PLUGIN)
        self.assertTrue(report["pass"], report)
        self.assertGreater(report["final"]["payload_displacement_m"][2], 0.045)
        self.assertLessEqual(report["runtime"]["maximum_tangent_residual"], 1e-5)
        self.assertLessEqual(report["runtime"]["maximum_surface_residual_m"], 1e-7)


if __name__ == "__main__":
    unittest.main()
