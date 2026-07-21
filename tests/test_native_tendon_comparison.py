import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PLUGIN_MODEL = ROOT / "cable_plugin_demos" / "17_cpp_plugin_three_strut_nine_cable.xml"
NATIVE_MODEL = ROOT / "cable_plugin_demos" / "18_native_tendon_three_strut_nine_cable.xml"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"),
    )
)


def load_comparison_script():
    path = ROOT / "scripts" / "compare_native_tendon_vs_rope.py"
    spec = importlib.util.spec_from_file_location("compare_native_tendon_vs_rope", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load script module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class NativeTendonComparisonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.comparison = load_comparison_script()

    def test_native_baseline_matches_plugin_topology_and_parameters(self):
        plugin_root = ET.parse(PLUGIN_MODEL).getroot()
        native_root = ET.parse(NATIVE_MODEL).getroot()

        self.assertEqual(len(native_root.findall(".//body/joint[@type='free']")), 3)
        self.assertEqual(native_root.findall("./extension"), [])
        self.assertEqual(native_root.findall("./actuator"), [])
        self.assertEqual(native_root.findall("./sensor"), [])
        self.assertEqual(native_root.findall(".//equality"), [])

        plugin_tendons = plugin_root.findall("./tendon/spatial")
        native_tendons = native_root.findall("./tendon/spatial")
        self.assertEqual(
            [tendon.attrib["name"] for tendon in native_tendons],
            [tendon.attrib["name"] for tendon in plugin_tendons],
        )
        self.assertEqual(len(native_tendons), 9)
        for tendon in native_tendons:
            self.assertEqual(float(tendon.attrib["stiffness"]), 1500.0)
            self.assertEqual(float(tendon.attrib["damping"]), 4.0)

    def test_initial_tension_matches_analytic_self_stress(self):
        model = mujoco.MjModel.from_xml_path(str(NATIVE_MODEL))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        rest = np.asarray(model.tendon_lengthspring[:, 0], dtype=float)
        tension = np.asarray(model.tendon_stiffness) * (
            np.asarray(data.ten_length) - rest
        )
        self_stress = self.comparison._self_stress(model)

        self.assertAlmostEqual(float(np.mean(tension[:6])), 1.45215, places=5)
        self.assertAlmostEqual(float(np.mean(tension[6:])), 3.0, places=5)
        self.assertAlmostEqual(
            float(np.mean(tension[:6]) / np.mean(tension[6:])),
            self_stress["ring_to_cross_ratio"],
            places=5,
        )
        self.assertLess(self_stress["node_equilibrium_residual_n"], 1e-12)

    def test_report_snapshot_is_read_back_by_declared_sql(self):
        datasets = {
            "metrics": [{"name": "tension", "value": 3.0}],
            "series": [{"time_s": 0.0, "value": 1.0}, {"time_s": 1.0, "value": 2.0}],
        }
        with tempfile.TemporaryDirectory() as directory:
            selected = self.comparison._materialize_report_snapshot(
                Path(directory), datasets
            )
            self.assertEqual(selected, datasets)
            self.assertTrue((Path(directory) / "report_snapshot.sqlite").exists())
            self.assertIn("SELECT dataset", self.comparison.REPORT_SOURCE_SQL)

    def test_portable_report_mobile_fallback_is_idempotent(self):
        path = ROOT / "scripts" / "finalize_portable_report.py"
        spec = importlib.util.spec_from_file_location("finalize_portable_report", path)
        if spec is None or spec.loader is None:
            self.fail(f"Could not load script module: {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report.html"
            report.write_text(
                '<html lang="en"><head></head><body></body></html>',
                encoding="utf-8",
            )
            self.assertTrue(module.finalize(report))
            self.assertFalse(module.finalize(report))
            html = report.read_text(encoding="utf-8")
            self.assertIn(module.STYLE_MARKER, html)
            self.assertIn(".portable-enhanced-hidden", html)
            self.assertIn('lang="zh-CN"', html)

    def test_unilateral_plugin_does_not_push_when_shorter_than_rest(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        rows = self.comparison._slack_benchmark(DEFAULT_PLUGIN)
        indexed = {(row["model"], row["state"]): row for row in rows}
        native = indexed[("MuJoCo native", "shorter_than_rest")]
        plugin = indexed[("Cable plugin", "shorter_than_rest")]
        self.assertAlmostEqual(native["applied_tension_n"], -10.0, places=10)
        self.assertAlmostEqual(plugin["applied_tension_n"], 0.0, places=10)
        self.assertAlmostEqual(plugin["generalized_force_n"], 0.0, places=10)


if __name__ == "__main__":
    unittest.main()
