# 下载与教程

## 可以直接下载什么

| 资源 | 下载/入口 | 包含与不包含 |
|---|---|---|
| 最小主动 surface | [XML](downloads/minimal_surface.xml) | 一个圆柱、自由物块、可控收绳，无外部 mesh |
| 最小被动 surface | [XML](downloads/minimal_passive.xml) | 无 actuator 的被动韧带式模型 |
| 最小 native | [XML](downloads/minimal_native.xml) | 原生路径与插件单边本构 |
| 14 组机构案例 | [案例库](gallery.md) | 各页含完整相对路径资产 ZIP、XML、Raw/数字视频 |
| 最新数值复跑 | [ZIP](downloads/latest-validation.zip) | CSV、JSON、图表、构建/运行哈希；非二进制运行时 |
| 全部视频清单 | [CSV](downloads/VIDEO_INVENTORY.csv) | 公开 examples 的归档目录，不是本机绝对路径 |
| 下载包校验清单 | [JSON](downloads/resource_manifest.json) | 模型包 SHA-256 与源文件路径 |

模型 ZIP 不是安装包。解压时保留目录结构；运行需要匹配的 MuJoCo、MuJoCable 动态库和 viewer。运行入口与依赖见[安装](installation.md)。

## 源码与发行

- [公开源码仓库](https://github.com/MuJoCable/mujoco-cable-dynamics)：安装、编译、测试、examples 与许可证。
- [GitHub Releases](https://github.com/MuJoCable/mujoco-cable-dynamics/releases)：检查某次发行是否提供你的 OS/架构/MuJoCo ABI 对应资产；不要把源代码 ZIP 当作预编译插件。
- [examples/paper](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples/paper)：论文相关冻结模型、统计与复跑入口。
- [examples/showcase](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples/showcase)：不规则轮和字母曲面示例。

源码和案例包保留 Apache-2.0、Faive 等来源说明。第三方模型的原始许可与署名条件见仓库 `examples/paper/ASSET_SOURCES.md`，不要把所有外部资产都重新声明成自己的原创。

## 既有教程与推导

下面提供已公开的原始文档下载；本网站的使用手册将它们按建模任务重新组织，而非简单罗列旧文件。

- {download}`Result 1：变量、方程与代数值推导 <downloads/RESULT1_CASE_THEORY_DERIVATIONS_ZH.md>`
- {download}`Result 1：理论与数值统计 <downloads/RESULT1_THEORY_AND_RESULTS_ZH.md>`
- {download}`Result 2：native 与插件建模差别 <downloads/RESULT2_MODEL_AND_METHOD_COMPARISON_ZH.md>`
- {download}`Result 2：关节与摩擦结果解释 <downloads/RESULT2_THEORY_AND_RESULTS_ZH.md>`
- [理论报告 PDF（中文）](downloads/plugin_method_strategy_report_zh.pdf) · [English PDF](downloads/plugin_method_strategy_report_en.pdf)：仓库既有报告；接口细节与本手册和对应源码交叉核对。
- [绳索设计与穿绳教程（中文）](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/CABLE_DESIGN_AND_THREADING_GUIDE_zh.md) · [English](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/CABLE_DESIGN_AND_THREADING_GUIDE.md)
- [Windows 安装与肘关节教程（中文）](https://mujocable.github.io/tutorials/windows/zh.html) · [English](https://mujocable.github.io/tutorials/windows/)
- [更多仓库文档](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/docs)

## 论文与引用

方法说明与研究边界以[论文 arXiv:2609.09612](https://arxiv.org/abs/2609.09612)及其修订为准。用于研究时，请引用论文并标明源码提交、MuJoCo 版本和实验模型。不要只报告一个未注明版本的项目主页链接。

```bibtex
@article{zhang2026mujocable,
  title={MuJoCable: Reduced-Order Surface-Routed Cable Transmission for Tendon-Driven Robots},
  author={Zhang, Yi and Shao, Qi and Lin, Yicong and Ma, Muyuan and Sun, Tao and Xie, Yue},
  journal={arXiv preprint arXiv:2609.09612},
  year={2026}
}
```
