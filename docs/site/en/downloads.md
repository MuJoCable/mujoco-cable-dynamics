# Downloads and Tutorials

## Available Resources

| Resource | Link | Contents and boundary |
|---|---|---|
| Minimal active surface model | <a href="../downloads/minimal_surface.xml">XML</a> | Cylinder, free payload and controllable take-up; no external mesh |
| Minimal passive surface model | <a href="../downloads/minimal_passive.xml">XML</a> | Passive cable without an actuator |
| Minimal native model | <a href="../downloads/minimal_native.xml">XML</a> | Native path with the plugin's unilateral law |
| 14 mechanisms | [Example gallery](gallery.md) | Screenshots, raw/annotated videos, complete relative-path model bundles |
| Current numerical reproductions | <a href="../downloads/latest-validation.zip">ZIP</a> | CSV, JSON, figures and source/library hashes; not a binary runtime |
| Complete video inventory | <a href="../downloads/VIDEO_INVENTORY.csv">CSV</a> | Public example archive paths, not personal working directories |
| Model-bundle checksums | <a href="../downloads/resource_manifest.json">JSON</a> | SHA-256 and source-model paths |

A model ZIP is not an application installer. Preserve the directory structure when extracting, then use a matching MuJoCo runtime, plugin library and viewer. See [installation](installation.md).

## Source and Releases

- [SolidWorks reference-geometry exporter](https://github.com/MuJoCable/solidworks-reference-geometry-exporter): define CAD landmarks, export CSV and bind MJCF sites; see the [routing tutorial](routing.md).

- [Public repository](https://github.com/MuJoCable/mujoco-cable-dynamics): source, build, tests, examples and licenses.
- [Releases](https://github.com/MuJoCable/mujoco-cable-dynamics/releases): check OS, architecture and MuJoCo ABI for each actual binary. A source ZIP is not a prebuilt plugin.
- [Paper examples](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples/paper): frozen models, results and reproduction entry points.
- [Showcase examples](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples/showcase): irregular pulleys and letter-mesh routes.

Bundles retain Apache-2.0 and relevant asset notices, including Faive attribution. Consult `examples/paper/ASSET_SOURCES.md` for third-party permissions; not every asset is an original MuJoCable design.

## Existing Reports and Guides

Original archived files are retained in their original language. They supplement the full English chapters; their inclusion is not a claim that all historical interfaces remain unchanged.

- <a href="../downloads/plugin_method_strategy_report_en.pdf">English theory report PDF</a> · <a href="../downloads/plugin_method_strategy_report_zh.pdf">Chinese PDF</a>.
- [Cable design and threading guide](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/CABLE_DESIGN_AND_THREADING_GUIDE.md) · [Chinese](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/CABLE_DESIGN_AND_THREADING_GUIDE_zh.md).
- [Windows installation and elbow tutorial](https://mujocable.github.io/tutorials/windows/) · [Chinese](https://mujocable.github.io/tutorials/windows/zh.html).
- <a href="../downloads/RESULT1_CASE_THEORY_DERIVATIONS_ZH.md">Result 1: equations and substituted numerical values (Chinese)</a>.
- <a href="../downloads/RESULT1_THEORY_AND_RESULTS_ZH.md">Result 1: theory and statistics (Chinese)</a>.
- <a href="../downloads/RESULT2_MODEL_AND_METHOD_COMPARISON_ZH.md">Result 2: native/plugin modeling comparison (Chinese)</a>.
- <a href="../downloads/RESULT2_THEORY_AND_RESULTS_ZH.md">Result 2: joints and friction analysis (Chinese)</a>.
- [Repository documentation](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/docs).

## Citation

For research use, cite [arXiv:2609.09612](https://arxiv.org/abs/2609.09612) and record the source commit, MuJoCo version and model. A versionless project-page URL alone is not a reproducible method specification.

```bibtex
@article{zhang2026mujocable,
  title={MuJoCable: Reduced-Order Surface-Routed Cable Transmission for Tendon-Driven Robots},
  author={Zhang, Yi and Shao, Qi and Lin, Yicong and Ma, Muyuan and Sun, Tao and Xie, Yue},
  journal={arXiv preprint arXiv:2609.09612},
  year={2026}
}
```
