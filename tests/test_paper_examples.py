"""Checks for the portable paper evidence and non-destructive run entrypoint."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import unittest

import mujoco


ROOT = Path(__file__).resolve().parents[1]
PAPER = ROOT / "examples/paper"


class PaperExamplesTest(unittest.TestCase):
    def test_asset_manifest(self):
        manifest = json.loads((PAPER / "ASSET_MANIFEST.json").read_text())
        self.assertGreater(len(manifest), 250)
        for relative, expected in manifest.items():
            with self.subTest(path=relative):
                path = PAPER / relative
                self.assertTrue(path.is_file())
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected)

    def test_archive_has_distinct_historical_and_latest_provenance(self):
        original = json.loads((PAPER / "PROVENANCE.json").read_text())
        current = json.loads((PAPER / "latest_plugin/RUN_MANIFEST.json").read_text())
        self.assertTrue(original["historical_results_are_not_latest_plugin_reruns"])
        self.assertFalse(current["fresh_media_rendered"])
        self.assertNotEqual(original["historical_plugin_sha256"], current["plugin_sha256"])
        self.assertEqual(len(list(PAPER.rglob("*.mp4"))), 38)

    def test_no_personal_paths_in_example_text(self):
        for path in PAPER.rglob("*"):
            if path.suffix not in {".md", ".xml", ".py", ".json", ".csv", ".svg", ".txt", ".obj"}:
                continue
            text = path.read_text()
            for token in ("/" + "Users/", "/opt/" + "anaconda", "/private/" + "tmp/"):
                self.assertNotIn(token, text, str(path.relative_to(ROOT)))

    def test_runner_refuses_to_overwrite_archive(self):
        spec = importlib.util.spec_from_file_location("paper_run_guard", PAPER / "run.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for output in (PAPER, PAPER / "new_output"):
            with self.assertRaises(ValueError):
                module.copy_inputs(output)

    def test_all_snapshot_models_load_and_step(self):
        library = os.environ.get("CABLE_PLUGIN_LIBRARY")
        if not library or not Path(library).is_file():
            self.skipTest("Set CABLE_PLUGIN_LIBRARY after compiling")
        mujoco.mj_loadPluginLibrary(library)
        models = sorted(PAPER.glob("result_*/**/*.xml"))
        self.assertEqual(len(models), 23)
        for path in models:
            with self.subTest(model=str(path.relative_to(PAPER))):
                model = mujoco.MjModel.from_xml_path(str(path))
                data = mujoco.MjData(model)
                for _ in range(3):
                    mujoco.mj_step(model, data)

    def test_showcase_models_load(self):
        library = os.environ.get("CABLE_PLUGIN_LIBRARY")
        if not library or not Path(library).is_file():
            self.skipTest("Set CABLE_PLUGIN_LIBRARY after compiling")
        mujoco.mj_loadPluginLibrary(library)
        paths = list((ROOT / "examples/showcase/models").glob("*.xml"))
        self.assertEqual(len(paths), 4)
        for path in paths:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            mujoco.mj_step(model, data)


if __name__ == "__main__":
    unittest.main()
