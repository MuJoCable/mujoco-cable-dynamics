# Third-Party Assets

## Faive Hand P0

`cable_plugin_demos/assets/faive_index_pip/index_pp.stl` and
`index_mp.stl` originate from the Faive Hand P0 assets in
[`srl-ethz/faive_gym_oss`](https://github.com/srl-ethz/faive_gym_oss).
Copyright 2023 Soft Robotics Lab, ETH Zurich. The assets are distributed under
Apache License 2.0; the original license text is retained next to the files.

The corresponding `*_outer.obj` and `*_contact.obj` files are deterministic
simulation derivatives of those source meshes. Their provenance is documented
in the asset directory.

## Saddle-Joint Assets

The saddle-joint meshes are project assets supplied for this research. Routed
and contact variants were generated from the supplied source geometry. Their
roles and regeneration notes are documented in
`cable_plugin_demos/assets/passive_saddle_joint/README.md`.

## 100_fingers Human-Hand Assets

The meshes under `cable_plugin_demos/assets/100_fingers_human/` are
deterministic connected-component and local-frame derivatives of
`human_hand_v3.1_nolig.stl` from the
[`kg398/100_fingers`](https://github.com/kg398/100_fingers)
parametric-hand project. The source project identifies its 3D assets as
licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
The original OpenSCAD implementation is GPLv3 and is not included in this
repository.

## OpenSpiRobs Log-Spiral Asset

`cable_plugin_demos/assets/open_spirob/spirob_unit_with_holes.stl` and the
derived Demo 33 MJCF models originate from
[`ZhanchiWang/Open-Spiral-Robots`](https://github.com/ZhanchiWang/Open-Spiral-Robots).
Copyright Zhanchi Wang. These files are distributed for noncommercial research
under PolyForm Noncommercial License 1.0.0. The license text is retained next
to the asset as
`cable_plugin_demos/assets/open_spirob/LICENSE.OpenSpiRobs-PolyForm-Noncommercial-1.0.0.txt`.
This notice does not change the Apache-2.0 license of the MuJoCable plugin code.
