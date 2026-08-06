import collections
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from scripts.analyze_eyelet_friction import (
    FRICTION_MODEL,
    ROOT,
    simulate_eyelet_friction,
    simulate_flex_aperture,
)


ASSET = ROOT / "cable_plugin_demos/assets/eyelet/washer.obj"
DEFAULT_PLUGIN = Path(
    os.environ.get(
        "CABLE_PLUGIN_LIBRARY",
        str(ROOT / "build/plugin/libcable_unilateral.dylib"),
    )
)


class EyeletFrictionTest(unittest.TestCase):
    def test_washer_is_closed_genus_one_mesh(self):
        vertices = []
        faces = []
        for line in ASSET.read_text(encoding="ascii").splitlines():
            if line.startswith("v "):
                vertices.append(tuple(float(value) for value in line.split()[1:]))
            elif line.startswith("f "):
                faces.append(tuple(int(value) - 1 for value in line.split()[1:]))
        edges = collections.Counter(
            tuple(sorted(edge))
            for face in faces
            for edge in (
                (face[0], face[1]),
                (face[1], face[2]),
                (face[2], face[0]),
            )
        )
        self.assertTrue(vertices)
        self.assertTrue(faces)
        self.assertTrue(all(count == 2 for count in edges.values()))
        euler_characteristic = len(vertices) - len(edges) + len(faces)
        self.assertEqual(euler_characteristic, 0)

    def test_rigid_flex_retains_aperture_while_mesh_geom_does_not(self):
        summary = simulate_flex_aperture()
        self.assertTrue(summary["pass"], summary)

    def test_eyelet_uses_role_three_guide_and_explicit_friction(self):
        root = ET.parse(FRICTION_MODEL).getroot()
        rough = root.find(
            "./extension/plugin/instance[@name='rough_eyelet_cable']"
        )
        self.assertIsNotNone(rough)
        configs = {
            node.attrib["key"]: node.attrib["value"]
            for node in rough.findall("config")
        }
        self.assertEqual(configs["route_mode"], "surface")
        self.assertEqual(configs["route_tendon"], "rough_eyelet_seed")
        self.assertEqual(configs["capstan_direction"], "forward")
        self.assertAlmostEqual(float(configs["guide_friction_mu"]), 0.45)
        self.assertNotIn("wrap_geoms", configs)

        seed = root.find("./tendon/spatial[@name='rough_eyelet_seed']")
        sites = [node.attrib["site"] for node in seed.findall("site")]
        self.assertEqual(
            sites, ["rough_anchor", "rough_guide", "rough_load_end"]
        )
        guide = root.find(".//site[@name='rough_guide']")
        guide_class = guide.attrib.get("class")
        default = root.find(f"./default/default[@class='{guide_class}']/site")
        self.assertEqual(default.attrib["user"], "3")

    def test_eyelet_tension_ratio_and_load_response(self):
        if not DEFAULT_PLUGIN.exists():
            self.skipTest(f"Compiled plugin not found: {DEFAULT_PLUGIN}")
        _, summary = simulate_eyelet_friction(DEFAULT_PLUGIN)
        self.assertTrue(summary["pass"], summary)
        self.assertGreater(summary["smooth_displacement_m"], 0.01)
        self.assertLess(summary["rough_displacement_m"], -0.004)
        self.assertAlmostEqual(
            summary["measured_rough_tension_ratio"],
            summary["expected_tension_ratio"],
            places=12,
        )


if __name__ == "__main__":
    unittest.main()
