# 不规则曲面展示

本目录是定性展示，不冒充论文的定量验证。包括椭圆、非对称凸轮、圆角长方体滑轮，
以及三根绳索依次包络不同字母表面的 ICRA 雕塑。全部模型与 raw/annotated 视频见
[English 索引和命令](README.md)。

2026-11-04 已用最新版插件按录制控制流程复跑四组数值轨迹，路线状态最大值均为 0，
详见 [latest_validation.json](latest_validation.json)。原视频仍作为历史录制保留。

字母厚度 24 mm，系绳点和 hint 位于厚度中心 y=0，绳路沿外轮廓侧壁包络，
不是在正面盖板上连线。只有 I 使用电机，C/R/A 通过被动 hinge 和绳力运动。
滑轮展示直接包络 mesh；长方体保留圆角，避免理想尖角处切线不唯一。

```bash
# 仓库根目录，macOS；Linux 改为 python 和 .so。
PLUGIN=build/plugin/libcable_unilateral.dylib

# 手动，Control 中自行调整 I_drive。
mjpython scripts/view_cpp_plugin_demo.py --plugin "$PLUGIN" \
  --model examples/showcase/models/icra_surface_cables.xml \
  --no-auto-control --show-cable-state --duration 120

# 自动施加转矩演示。
mjpython scripts/view_cpp_plugin_demo.py --plugin "$PLUGIN" \
  --model examples/showcase/models/icra_surface_cables.xml \
  --auto-actuator I_drive --ctrl-final 0.008 --duration 120
```

ICRA 字母仅用于学术展示，不代表官方素材或背书。落地、松弛不等于该无质量模型
能够预测绳子的下垂及脱离后的重新接触；目前仍固定初始化的路线分支。
