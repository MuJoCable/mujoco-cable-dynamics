#!/usr/bin/env python3
"""Validate that the public release tree is portable and intentionally scoped."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_DEMOS = {
    "09_cpp_plugin_free_hanging_single_pulley.xml",
    "10_cpp_plugin_dual_pulley_free_payload.xml",
    "11_cpp_plugin_reverse_reserve_release.xml",
    "12_cpp_plugin_frictional_pulley_free_payload.xml",
    "13_cpp_plugin_rolling_joint_figure_eight.xml",
    "14_cpp_plugin_convex_mesh_rolling_joint.xml",
    "15_cpp_plugin_surface_single_pulley.xml",
    "16_cpp_plugin_passive_saddle_joint.xml",
    "17_cpp_plugin_three_strut_nine_cable.xml",
    "18_native_tendon_three_strut_nine_cable.xml",
    "19_cpp_plugin_mixed_stiffness_tensegrity.xml",
    "20_cpp_plugin_controlled_saddle_joint.xml",
    "21_cpp_plugin_wheel_axle_force_amplifier.xml",
    "24_faive_index_pip_virtual_hinge_baseline.xml",
    "25_faive_index_pip_surface_cable.xml",
}
FORBIDDEN_DIRECTORIES = {
    ".DS_Store",
    ".AppleDouble",
    ".Spotlight-V100",
    ".Trashes",
    ".agents",
    ".claude",
    ".openai",
    ".playwright-cli",
    "." + "codex",
    "__pycache__",
    "node_modules",
}
FORBIDDEN_TEXT = (
    "/" + "Users/",
    "/opt/" + "anaconda",
    "/private/" + "tmp/",
    "clock" + "yee",
    "mujoco-cable-dynamics-plugin-" + "github-ready",
)
TEXT_SUFFIXES = {
    ".cc",
    ".cmake",
    ".h",
    ".json",
    ".md",
    ".py",
    ".sh",
    ".txt",
    ".xml",
    ".yml",
}


def main() -> int:
    errors: list[str] = []
    actual_demos = {path.name for path in (ROOT / "cable_plugin_demos").glob("*.xml")}
    if actual_demos != EXPECTED_DEMOS:
        errors.append(
            "demo set mismatch: "
            f"missing={sorted(EXPECTED_DEMOS - actual_demos)}, "
            f"extra={sorted(actual_demos - EXPECTED_DEMOS)}"
        )

    for path in ROOT.rglob("*"):
        relative = path.relative_to(ROOT)
        if ".git" in relative.parts:
            continue
        for part in relative.parts:
            if part in FORBIDDEN_DIRECTORIES or part.startswith("._"):
                errors.append(f"forbidden local artifact: {relative}")
                break
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for token in FORBIDDEN_TEXT:
            if token in text:
                errors.append(f"developer-specific text in {relative}: {token}")

    if errors:
        print("Release-tree validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print(f"Release tree is portable and contains {len(actual_demos)} selected demos.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
