# Result 2.2c：Demo 42 单输入六索手指

## 作用

该辅助实验把 Demo 42 的三关节仿生手指纳入 Result 2.2，用于展示 MuJoCable
在另一类多关节机构中的运动与摩擦敏感性。它不改变 Result 2.2a/b 的主对照：
主对照仍是相同柔性参数下的 native tendon 与 MuJoCable 对数螺旋机器人。

手指包含 MCP、PIP、DIP 三个 hinge。Route 1 与 Route 4 由一个差动输入反向
收放，其余四条绳为被动耦合索。基线参数为 guide friction `mu_g=0.20`、
pulley friction `mu_p=0.05`。

## 视频

- `videos/result_2_2c_demo42_oblique_raw.mp4`：完整斜视图，无文字；
- `videos/result_2_2c_demo42_oblique_annotated.mp4`：完整斜视图，带状态；
- `videos/result_2_2c_demo42_bending_plane_raw.mp4`：完整弯曲平面正视图，无文字；
- `videos/result_2_2c_demo42_bending_plane_annotated.mp4`：完整弯曲平面正视图，带状态。

斜视图和弯曲平面视图不再左右拼接，而是分别使用完整 `960 x 540` 画幅。斜视图
使用完整机构包围盒取景，正视图把相机视场扩大到 `65 deg`。初始伸直、正向完全弯曲、反向
切换和最终回程均需保持基座与指尖完整入画。

## 复用的摩擦结果

`data/` 和 `figures/` 直接复用此前 Demo 42 的同模型扫描结果，没有重复生成
或修改数值。

| 扫描 | 固定参数 | 变化参数 | 主要输出 |
|---|---|---|---|
| Guide/孔口摩擦 | `mu_p=0.05` | `mu_g=0,0.05,0.10,0.20,0.30` | 三关节角、总弯曲速度、Route 2 张力 |
| Pulley/Capstan 摩擦 | `mu_g=0.20` | `mu_p=0,0.025,0.05,0.10,0.20` | 三关节角、总弯曲速度、Route 1 张力 |

代表性结果：`mu_g` 从 `0` 增至 `0.30` 时，3 s 总弯曲由 `269.27 deg`
降至 `214.35 deg`；`mu_p` 从 `0` 增至 `0.20` 时，3 s 总弯曲由
`251.62 deg` 降至 `146.22 deg`，而 Route 1 峰值张力升至 `25 N` 上限。
这说明较高摩擦会降低远端可用张力和固定时间内的弯曲量，但这些系数尚未经过
实物辨识，因此结论属于仿真敏感性分析。

## 文件目录

```text
demo42_single_finger/
|- models/       可移植 MJCF
|- videos/       两个独立视角的 raw、annotated 视频和视频摘要
|- data/         基线时序、摩擦扫描时序及汇总
|- figures/      IEEE 风格 PDF/SVG/600 dpi PNG
`- README_ZH.md  本说明
```

打开可交互模型：

```bash
conda activate rope_plugin

mjpython "[your path]/mujoco-cable-dynamics/scripts/view_cpp_plugin_demo.py" \
  --plugin "[your path]/mujoco-cable-dynamics/build/plugin/libcable_unilateral.dylib" \
  --model "[your path]/experiment0814/result_2_2/demo42_single_finger/models/result_2_2c_demo42_single_finger.xml" \
  --show-cable-state --duration 120
```
