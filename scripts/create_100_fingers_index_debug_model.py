#!/usr/bin/env python3
"""Create an index-only cable-debug variant of the 100-fingers hand model."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT / "cable_plugin_demos/26_cpp_plugin_100_fingers_human_hand.xml"
)
DEFAULT_OUTPUT = (
    ROOT / "cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml"
)
DIGITS = ("index", "middle", "ring", "little", "thumb")


def _digit_from_name(name: str) -> str | None:
    return next((digit for digit in DIGITS if name.startswith(f"{digit}_")), None)


def _remove_inactive_children(parent: ET.Element | None, active_digit: str) -> None:
    if parent is None:
        return
    for child in list(parent):
        digit = _digit_from_name(child.attrib.get("name", ""))
        if digit is not None and digit != active_digit:
            parent.remove(child)


def make_debug_model(
    source: Path,
    output: Path,
    active_digit: str = "index",
) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "100_fingers_index_cable_debug")

    _remove_inactive_children(root.find("./extension/plugin"), active_digit)
    _remove_inactive_children(root.find("./tendon"), active_digit)
    _remove_inactive_children(root.find("./actuator"), active_digit)

    retained_route_sites = {
        site.attrib["site"]
        for tendon in root.findall("./tendon/spatial")
        for site in tendon.findall("site")
    }

    sensor = root.find("./sensor")
    if sensor is not None:
        for child in list(sensor):
            if child.tag != "plugin":
                continue
            digit = _digit_from_name(child.attrib.get("name", ""))
            if digit is not None and digit != active_digit:
                sensor.remove(child)

    # Remove cable-only debug geometry from inactive digits. Physical bodies,
    # joints, meshes, and fingertip sensors remain unchanged.
    for parent in root.iter():
        for child in list(parent):
            name = child.attrib.get("name", "")
            digit = _digit_from_name(name)
            if digit is None:
                continue
            if (
                child.tag == "site"
                and "user" in child.attrib
                and (
                    digit != active_digit
                    or name not in retained_route_sites
                )
            ):
                parent.remove(child)
            elif (
                child.tag == "geom"
                and digit != active_digit
                and name.endswith("_wrap")
            ):
                parent.remove(child)

    worldbody = root.find("./worldbody")
    if worldbody is None:
        raise ValueError("Source model has no worldbody")
    if worldbody.find("./camera[@name='index_route_debug']") is None:
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "index_route_debug",
                "pos": "0.050 -0.220 0.197",
                "xyaxes": "1 0 0 0 0 1",
            },
        )

    expected = 5
    counts = {
        "instances": len(root.findall("./extension/plugin/instance")),
        "tendons": len(root.findall("./tendon/spatial")),
        "actuators": len(root.findall("./actuator/plugin")),
        "plugin sensors": len(root.findall("./sensor/plugin")),
    }
    wrong = {name: count for name, count in counts.items() if count != expected}
    if wrong:
        raise ValueError(f"Unexpected index-only model counts: {wrong}")

    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--active-digit", choices=DIGITS, default="index")
    args = parser.parse_args()
    make_debug_model(
        args.source.expanduser().resolve(),
        args.output.expanduser().resolve(),
        args.active_digit,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
