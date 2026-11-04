# 论文案例与复现

[English](README.md) | [论文](https://arxiv.org/abs/2609.09612)

本目录把论文案例按照“理论—仿真”“仿真—仿真”“实物配套仿真”组织起来。
历史 CSV、图表和视频原样归档；新版插件复跑写入 `outputs/paper_latest/`，不覆盖
旧数据。原始插件哈希和文件来源记录在 [PROVENANCE.json](PROVENANCE.json)。

本次还提供[新版插件的实际复跑记录](latest_plugin/README.md)，包括滑轮、Faive、
五组摩擦对照与完整弯曲回正周期的新 CSV/图表及源码哈希。视频仍明确标注为历史录制。

| 类别 | 案例 | 主要材料 |
|---|---|---|
| 理论—仿真 | Result 1.1、1.1b：固定/自由滑轮，mu=0/0.15 | 张力、包角、角速度、Capstan 和转矩方程 |
| 理论—仿真 | Result 1.1c：卷轴；Result 1.2：动滑轮 | 圆周位移、半径比、两承载绳段与提升关系 |
| 理论—仿真 | Result 1.3：Atwood 落地 | 落地前速度，落地后碰撞与松弛 |
| 仿真对照 | Result 2.1：Faive 滚动关节 | 双虚拟铰链与真实 mesh 绳路、50 度运动 |
| 仿真对照 | Result 2.2：SpiRobs | 匹配关节弹性的 native tendon/muscle 与 MuJoCable |
| 摩擦干预 | Result 2.3 | mu=0、0.15、0.30、0.45、0.60 的收绳量、张力及逐关节角 |
| 单指 | Result 2.2c / 3.1 的 Demo 42 | 三关节六索、差动驱动、孔口/滑轮摩擦扫描、独立视角视频 |
| 配套仿真 | Result 3.2 / Demo 49 | 右弯—回正—左弯—回正、20 mm 卷线盘角度表 |

所有模型、视频和结果路径见 [English 索引](README.md)、[VIDEO_INVENTORY.csv](VIDEO_INVENTORY.csv)。
Result 3.1 与 Demo 42 重复的结果只存一份。**没有上传实物原始重复试验数据**，
不能把这些仿真文件表述成完整实物标定集。

## 打开与复跑

先按照仓库 README 编译最新版插件，然后在仓库根目录执行：

```bash
python -m pip install -r examples/paper/requirements.txt
python examples/paper/run.py list

# 人工控制；macOS 自动选择 mjpython，默认不自动收绳。
python examples/paper/run.py view rolling_mesh --show-cable-state
python examples/paper/run.py view finger --show-cable-state
python examples/paper/run.py view spirobs --show-cable-state
python examples/paper/run.py view moving_pulley

# 新版数值实验，保存到独立目录。
python examples/paper/run.py result1 --skip-videos
python examples/paper/run.py result2 --skip-videos

# 标定柔性版本：右弯—回正—左弯—回正，输出 20 mm 绕线盘舵机表。
python examples/paper/run.py cycle --duration 24 --guide-friction-mu 0.015 \
  --output outputs/paper_cycle

# 录制一个滑轮案例，需要 FFmpeg 和可用 OpenGL。
python examples/paper/run.py result1 --case result_1_1_mu_0p15 \
  --output outputs/paper_fixed_pulley

python examples/paper/run.py audit --check-videos \
  --report outputs/paper_latest/model_audit.json
```

可用 `--plugin` 或环境变量 `CABLE_PLUGIN_LIBRARY` 指定动态库；默认从当前仓库
`build/plugin/` 选择 macOS 的 `.dylib` 或 Linux 的 `.so`。结果复跑记录动态库
及核心源码 SHA-256。字体优先使用已安装的 Times New Roman，无此字体时使用 serif。

SpiRobs 有初始余绳，少量拖动滑块可能仍未绷紧；论文轨迹应使用复现控制器。
普通手指、滚动关节以绳驱动为主；双虚拟 hinge/native baseline 和卷轴电机例外，
不能将理想关节基线也称为纯绳驱动。

## 解释边界

卷轴案例使用 equality 构造理想半径比，不是由摩擦自动产生的无滑移约束。
动滑轮允许滑移，其转角比不应直接视作理想理论值。Atwood 理论曲线只适用于落地前。
Faive 50 度案例不等于已验证完整被动韧带网络在所有姿态均有效。
摩擦系数扫描属于仿真敏感性分析，不等于已完成实物参数辨识。

保留原有统计中的饱和、失败和路线有效率，不将其改写为全程成功。详细说明见
[滑轮理论与结果](RESULT1_THEORY_AND_RESULTS_ZH.md)、
[Result 2 方法对照](RESULT2_MODEL_AND_METHOD_COMPARISON_ZH.md) 和各案例 JSON。
素材来源与授权见 [ASSET_SOURCES.md](ASSET_SOURCES.md)。引用请使用仓库 README 的
MuJoCable 论文条目。
