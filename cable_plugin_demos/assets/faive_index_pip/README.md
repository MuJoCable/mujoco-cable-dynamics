# Faive index PIP reference assets

`index_pp.stl` and `index_mp.stl` are extracted without geometric modification
from the Faive Hand P0 model distributed in `faive_gym_oss`:

- Project: <https://srl-ethz.github.io/get-ball-rolling/>
- Source: <https://github.com/srl-ethz/faive_gym_oss>
- Archived commit in the local reference ZIP: `e46ae11fe3f52619ba52284adf8a34e92ed73a14`
- Copyright: 2023 Soft Robotics Lab, ETH Zurich
- License: Apache License 2.0

The license text is included as `LICENSE.faive-apache-2.0.txt`.

The comparison demos preserve the original mesh scale (`0.001`) and the two
rolling-surface axis locations reported in the Faive MJCF. The source STL
triangle soups are nonmanifold. `prepare_faive_pip_outer_surfaces.py` extracts
closed, consistently oriented outer shells and conformingly subdivides route
edges longer than `2 mm`; this keeps a seed triangle from spanning the full
finger width. Open low-face-count patches from the unrefined shells provide
MuJoCo rigid-flex contact. The visual STL, contact patch, and closed cable-route
shell therefore share the source geometry but have separate simulation roles.
