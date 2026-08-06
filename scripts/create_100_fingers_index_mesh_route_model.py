#!/usr/bin/env python3
"""Create the actual-mesh routing and passive-ligament index debug model."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT / "cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml"
)
DEFAULT_OUTPUT = (
    ROOT
    / "cable_plugin_demos/28_cpp_plugin_100_fingers_index_mesh_threading.xml"
)
SEGMENTS = ("proximal", "intermediate", "distal")
ACTIVE_ROUTES = {
    "index_extensor_cable": (
        "index_proximal_surface index_intermediate_surface index_distal_surface",
        "0 0 1",
    ),
    "index_abductor_cable": ("index_proximal_surface", "0 1 0"),
    "index_adductor_cable": ("index_proximal_surface", "0 1 0"),
    "index_flexor_intermediate_cable": (
        "index_proximal_surface index_intermediate_surface",
        "0 0 1",
    ),
    "index_flexor_distal_cable": (
        "index_proximal_surface index_intermediate_surface index_distal_surface",
        "0 0 1",
    ),
}
LIGAMENTS = (
    # name, parent body/position, child body/hint/end, child wrap surface
    ("mcp_left", "index_mount", "-0.004 0.010 0.0048",
     "index_proximal", "0 0.010 0.0048", "0.004 0.010 0.0048",
     "index_proximal_surface"),
    ("mcp_right", "index_mount", "-0.004 0.010 -0.0048",
     "index_proximal", "0 0.010 -0.0048", "0.004 0.010 -0.0048",
     "index_proximal_surface"),
    ("pip_left", "index_proximal", "0.0285 0.0092 0.0042",
     "index_intermediate", "0 0.0092 0.0042", "0.004 0.0092 0.0042",
     "index_intermediate_surface"),
    ("pip_right", "index_proximal", "0.0285 0.0092 -0.0042",
     "index_intermediate", "0 0.0092 -0.0042", "0.004 0.0092 -0.0042",
     "index_intermediate_surface"),
    ("dip_left", "index_intermediate", "0.0165 0.0078 0.0034",
     "index_distal", "0 0.0078 0.0034", "0.0035 0.0078 0.0034",
     "index_distal_surface"),
    ("dip_right", "index_intermediate", "0.0165 0.0078 -0.0034",
     "index_distal", "0 0.0078 -0.0034", "0.0035 0.0078 -0.0034",
     "index_distal_surface"),
)


def _config(instance: ET.Element) -> dict[str, ET.Element]:
    return {item.attrib["key"]: item for item in instance.findall("config")}


def _body(root: ET.Element, name: str) -> ET.Element:
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise ValueError(f"Body not found: {name}")
    return body


def _add_ligaments(root: ET.Element) -> None:
    plugin = root.find("./extension/plugin")
    tendon = root.find("./tendon")
    sensor = root.find("./sensor")
    if plugin is None or tendon is None or sensor is None:
        raise ValueError("Source model is missing plugin, tendon, or sensor")

    for (
        short_name,
        parent_name,
        parent_pos,
        child_name,
        hint_pos,
        child_pos,
        wrap_geom,
    ) in LIGAMENTS:
        name = f"index_ligament_{short_name}"
        start = f"{name}_start"
        hint = f"{name}_hint"
        end = f"{name}_end"
        ET.SubElement(
            _body(root, parent_name),
            "site",
            name=start,
            pos=parent_pos,
            user="1",
            size="0.00045",
            rgba="0.94 0.82 0.22 1",
        )
        ET.SubElement(
            _body(root, child_name),
            "site",
            name=hint,
            pos=hint_pos,
            user="2",
            size="0.0003",
            rgba="0.94 0.82 0.22 0",
        )
        ET.SubElement(
            _body(root, child_name),
            "site",
            name=end,
            pos=child_pos,
            user="1",
            size="0.00045",
            rgba="0.94 0.82 0.22 1",
        )
        instance = ET.SubElement(plugin, "instance", name=f"{name}_cable")
        values = {
            "route_mode": "surface",
            "route_tendon": f"{name}_seed",
            "wrap_geoms": wrap_geom,
            "mesh_route_mode": "guided_surface",
            "mesh_guide_axis": "0 0 1",
            "mesh_guide_weight": "6",
            "home_length": "auto_initial",
            "stiffness": "180",
            "damping": "0.35",
            "slack": "0.0001",
            "pretension_offset": "0.00015",
            "max_tension": "2.0",
            "taut_transition": "0.00008",
            "taut_hysteresis": "0.00002",
            "visual_width": "1.0",
            "visual_smoothing_timeconstant": "0.02",
        }
        for key, value in values.items():
            ET.SubElement(instance, "config", key=key, value=value)
        spatial = ET.SubElement(
            tendon,
            "spatial",
            name=f"{name}_seed",
            width="0.000000001",
            rgba="0.94 0.82 0.22 1",
            limited="false",
        )
        ET.SubElement(spatial, "site", site=start)
        ET.SubElement(spatial, "site", site=hint)
        ET.SubElement(spatial, "site", site=end)
        ET.SubElement(
            sensor,
            "plugin",
            name=f"{name}_state",
            instance=f"{name}_cable",
        )


def create(source: Path, output: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "100_fingers_index_mesh_threading")

    asset = root.find("./asset")
    if asset is None:
        raise ValueError("Source model has no asset section")
    for segment in SEGMENTS:
        ET.SubElement(
            asset,
            "mesh",
            name=f"index_{segment}_route_mesh",
            file=f"human_index_{segment}_route.obj",
        )

    for segment in SEGMENTS:
        geom = root.find(f".//geom[@name='index_{segment}_visual']")
        if geom is None:
            raise ValueError(f"Index {segment} visual geom not found")
        geom.set("name", f"index_{segment}_surface")
        geom.set("mesh", f"index_{segment}_route_mesh")

    # The cylinder baseline used persistent role-3 points to imitate printed
    # guides. In the mesh-threading model the pulley/groove surface itself is
    # the guide, so these fixed kinks must not remain in the runtime route.
    for spatial in root.findall("./tendon/spatial"):
        for site_ref in list(spatial):
            if site_ref.attrib.get("site", "").endswith("_guide"):
                spatial.remove(site_ref)
    for parent in root.iter():
        for child in list(parent):
            if (
                child.tag == "site"
                and child.attrib.get("name", "").endswith("_guide")
                and child.attrib.get("user") == "3"
            ):
                parent.remove(child)

    for parent in root.iter():
        for child in list(parent):
            if (
                child.tag == "geom"
                and child.attrib.get("name", "").startswith("index_")
                and child.attrib.get("name", "").endswith("_wrap")
            ):
                parent.remove(child)

    for instance_name, (wrap_geoms, guide_axis) in ACTIVE_ROUTES.items():
        instance = root.find(
            f"./extension/plugin/instance[@name='{instance_name}']"
        )
        if instance is None:
            raise ValueError(f"Plugin instance not found: {instance_name}")
        config = _config(instance)
        config["wrap_geoms"].set("value", wrap_geoms)
        ET.SubElement(
            instance, "config", key="mesh_route_mode", value="guided_surface"
        )
        ET.SubElement(
            instance, "config", key="mesh_guide_axis", value=guide_axis
        )
        ET.SubElement(
            instance, "config", key="mesh_guide_weight", value="6"
        )

    _add_ligaments(root)
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    create(args.source.expanduser().resolve(), args.output.expanduser().resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
