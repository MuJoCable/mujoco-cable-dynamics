from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEMO_DIR = ROOT / "cable_plugin_demos"


class PulleySurfaceRouteTest(unittest.TestCase):
    def test_winch_demos_use_hidden_surface_seeds(self):
        expected_wraps = {
            "09_cpp_plugin_free_hanging_single_pulley.xml": [
                "winch_drum_visual",
                "top_pulley_visual",
            ],
            "10_cpp_plugin_dual_pulley_free_payload.xml": [
                "winch_drum_visual",
                "left_pulley_visual",
                "right_pulley_visual",
            ],
            "11_cpp_plugin_reverse_reserve_release.xml": [
                "winch_drum_visual",
                "top_pulley_visual",
            ],
        }
        for filename, wrap_geoms in expected_wraps.items():
            with self.subTest(filename=filename):
                root = ET.parse(DEMO_DIR / filename).getroot()
                configs = {
                    config.attrib["key"]: config.attrib["value"]
                    for config in root.findall(".//extension/plugin/instance/config")
                }
                self.assertEqual(configs["route_mode"], "surface")
                self.assertEqual(configs["wrap_geoms"].split(), wrap_geoms)

                tendon = root.find(
                    f".//tendon/spatial[@name='{configs['route_tendon']}']"
                )
                self.assertIsNotNone(tendon)
                self.assertLess(float(tendon.attrib["width"]), 1e-6)
                self.assertEqual(tendon.findall("geom"), [])

                site_names = [element.attrib["site"] for element in tendon]
                sites = {
                    site.attrib["name"]: site
                    for site in root.findall(".//site")
                    if "name" in site.attrib
                }
                roles = [int(float(sites[name].attrib.get("user", "0"))) for name in site_names]
                self.assertEqual(roles[0], 1)
                self.assertEqual(roles[-1], 1)
                self.assertEqual(roles.count(2), len(wrap_geoms))

                actuator = root.find(".//actuator/plugin")
                self.assertIsNotNone(actuator)
                self.assertEqual(actuator.attrib.get("gear"), "0")

    def test_dual_pulley_route_has_no_fixed_bridge_site(self):
        root = ET.parse(
            DEMO_DIR / "10_cpp_plugin_dual_pulley_free_payload.xml"
        ).getroot()
        self.assertIsNone(root.find(".//site[@name='site_bridge_between_pulleys']"))


if __name__ == "__main__":
    unittest.main()
