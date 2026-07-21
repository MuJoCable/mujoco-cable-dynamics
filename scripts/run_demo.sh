#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEMO="${1:-15}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "$DEMO" in
  09) MODEL="$ROOT/cable_plugin_demos/09_cpp_plugin_free_hanging_single_pulley.xml" ;;
  10) MODEL="$ROOT/cable_plugin_demos/10_cpp_plugin_dual_pulley_free_payload.xml" ;;
  11) MODEL="$ROOT/cable_plugin_demos/11_cpp_plugin_reverse_reserve_release.xml" ;;
  12) MODEL="$ROOT/cable_plugin_demos/12_cpp_plugin_frictional_pulley_free_payload.xml" ;;
  13) MODEL="$ROOT/cable_plugin_demos/13_cpp_plugin_rolling_joint_figure_eight.xml" ;;
  14) MODEL="$ROOT/cable_plugin_demos/14_cpp_plugin_convex_mesh_rolling_joint.xml" ;;
  15) MODEL="$ROOT/cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml" ;;
  16) MODEL="$ROOT/cable_plugin_demos/16_cpp_plugin_passive_saddle_joint.xml" ;;
  17) MODEL="$ROOT/cable_plugin_demos/17_cpp_plugin_three_strut_nine_cable.xml" ;;
  18) MODEL="$ROOT/cable_plugin_demos/18_native_tendon_three_strut_nine_cable.xml" ;;
  19) MODEL="$ROOT/cable_plugin_demos/19_cpp_plugin_mixed_stiffness_tensegrity.xml" ;;
  20) MODEL="$ROOT/cable_plugin_demos/20_cpp_plugin_controlled_saddle_joint.xml" ;;
  21) MODEL="$ROOT/cable_plugin_demos/21_cpp_plugin_wheel_axle_force_amplifier.xml" ;;
  24) MODEL="$ROOT/cable_plugin_demos/24_faive_index_pip_virtual_hinge_baseline.xml" ;;
  25) MODEL="$ROOT/cable_plugin_demos/25_faive_index_pip_surface_cable.xml" ;;
  *) MODEL="" ;;
esac

if [[ -z "$MODEL" && -f "$DEMO" ]]; then
  MODEL="$(cd "$(dirname "$DEMO")" && pwd)/$(basename "$DEMO")"
elif [[ -z "$MODEL" ]]; then
  echo "Unknown demo '$DEMO'. Available: 09 10 11 12 13 14 15 16 17 18 19 20 21 24 25" >&2
  exit 2
fi

PLUGIN="${MUJOCABLE_PLUGIN:-}"
if [[ -z "$PLUGIN" ]]; then
  for candidate in \
    "$ROOT/lib/libcable_unilateral.dylib" \
    "$ROOT/lib/libcable_unilateral.so" \
    "$ROOT/build/plugin/libcable_unilateral.dylib" \
    "$ROOT/build/plugin/libcable_unilateral.so"; do
    if [[ -f "$candidate" ]]; then
      PLUGIN="$candidate"
      break
    fi
  done
fi

if [[ -z "$PLUGIN" || ! -f "$PLUGIN" ]]; then
  echo "Cable plugin library not found. Set MUJOCABLE_PLUGIN or build the project." >&2
  exit 2
fi

if [[ "$(uname -s)" == "Darwin" ]]; then
  RUNNER="${MUJOCO_PYTHON:-mjpython}"
else
  RUNNER="${MUJOCO_PYTHON:-python3}"
fi

exec "$RUNNER" "$ROOT/scripts/view_cpp_plugin_demo.py" \
  --plugin "$PLUGIN" --model "$MODEL" --show-cable-state "$@"
