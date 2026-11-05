# Asset Sources and Permissions

- **MuJoCable code and experiment definitions:** Apache-2.0, subject to the
  repository license. Numeric records and author-generated plots/videos are
  provided as supporting research materials; cite the MuJoCable paper.
- **Faive P0 index meshes:** Soft Robotics Lab, ETH Zurich, Apache-2.0.
  Source project: https://github.com/srl-ethz/faive_gym_oss . The source commit
  and full license are retained in `assets/faive_index_pip/`.
- **Single-finger geometry:** user-supplied SolidWorks assembly exports;
  public redistribution was explicitly authorized by the project owner on
  2026-11-04. No original SolidWorks or STEP assemblies are included. The
  coordinate registration and mesh/contact-patch roles are documented in
  `assets/biomimetic_finger/README.md`.
- **SpiRobs geometry:** user-supplied prototype geometry represented by
  deterministic exported module meshes. Public redistribution was explicitly
  authorized by the project owner on 2026-11-04. The geometric extraction and
  nominal/calibrated flexibility assumptions are retained in the asset
  manifest and the corresponding model.
- **Native SpiRobs baseline:** the one-DOF hinge and spatial-tendon/muscle
  construction follows the public Open-Spiral-Robots modeling pattern:
  https://github.com/ZhanchiWang/Open-Spiral-Robots . This is an adapted,
  parameter-matched simulation baseline, not a claim to redistribute all of
  that project's CAD or to reproduce every original controller.

No third-party paper PDFs, commercial fonts, original CAD assemblies, private
hardware trial datasets or machine-specific binary runtimes are bundled.
Author-owned material is distributed under the repository's Apache-2.0 terms;
the third-party license notices remain in force for their respective files.
