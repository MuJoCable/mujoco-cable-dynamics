# MuJoCo 绳索动力学插件

[English README](README.md) | [学术项目主页](https://mujocable.github.io/) |
[精选示例](docs/DEMO_CATALOG.md)

这是一个独立的 MuJoCo C++ engine plugin，用于模拟在圆柱和闭合 mesh 表面上包络的
无质量单边受拉绳索。插件不把绳子离散成大量刚体，而是在运行时计算绳长、自由长度、
张力、表面接触点以及作用于刚体的力和力矩。

![MuJoCo 绳索动力学示例：表面绳索滚动关节、三杆九索张拉整体和运行时表面包络滑轮](cable_plugin_demos/screenshots/readme_mujoco_overview.gif)

*MuJoCo 仿真动图：Faive PIP 表面绳索滚动关节（左）、三杆九索张拉整体（右上）和
运行时表面包络滑轮（右下）。*

## 主要能力

- 含松弛、预张紧、阻尼和最大张力的单边 Kelvin-Voigt 绳索本构；
- 收缩量、卷轴角度、卷轴速度和真实卷轴关节控制；
- 有符号预绕储备和可选卷轴反力矩；
- 解析圆柱包络与闭合凸/非凸 mesh 的同伦引导表面路线；
- 多个运动表面之间的复合包络与公切线连接；
- 可选 Euler-Eytelwein/Capstan 分段张力传播；
- 被动韧带、主动绳索、实时传感器与标准 MuJoCo scene 可视化；
- 用于减小切换抖动的路线、本构和显示迟滞。

适用范围是可以忽略绳质量、下垂、弯扭、波传播和自接触的准静态或低频绳驱机构。

## 与 MuJoCo 原生 Tendon 的关系

普通固定 site 路径和 MuJoCo 已支持的 wrap object 应优先使用原生 spatial tendon。
本插件补充以下机制建模能力：

| 能力 | 原生 tendon | 本插件 |
|---|---:|---:|
| 单边松弛本构 | 需要针对模型处理 | 内置 |
| 在线自由绳长控制 | 有限 | 内置 |
| 卷轴角与预绕储备 | 无直接绳索状态 | 内置 |
| 闭合 mesh 障碍包络 | 有限 | 支持凸面和引导非凸面 |
| 多运动表面复合路线 | 有限 | 保存公切线路线状态 |
| 路线状态和残差传感器 | 无 | 有 |

Demo 17 和 Demo 18 提供了参数匹配的插件/原生 tendon 张拉整体对照。

## 快速使用二进制 Release

[Releases 页面](https://github.com/MuJoCable/mujoco-cable-dynamics/releases)提供对应操作
系统和 CPU 的压缩包。二进制按照 MuJoCo `3.4.x` 构建，请先安装匹配的 Python runtime：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install "mujoco>=3.4,<3.5" numpy
```

下载并解压对应平台的压缩包后：

```bash
cd mujoco-cable-dynamics-v0.1.1-<platform>-<architecture>
./scripts/run_demo.sh 15 --show-route-debug --duration 120
```

macOS 启动器会使用 `mjpython`，因为 MuJoCo 被动 viewer 要求使用该入口；Linux 使用
`python3`。压缩包可以在相同操作系统、CPU 架构和 MuJoCo `3.4.x` ABI 范围内直接
使用，但不是完全静态、零依赖的单文件应用。

## 从源码构建

### 1. 下载仓库并安装环境

```bash
git clone https://github.com/MuJoCable/mujoco-cable-dynamics.git
cd mujoco-cable-dynamics

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

也可以在已有的 conda 环境中执行 `python -m pip install -e .`。

### 2. 编译插件

```bash
MUJOCO_DIR="$(python -c 'import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release
```

默认输出为：

- macOS：`build/plugin/libcable_unilateral.dylib`
- Linux：`build/plugin/libcable_unilateral.so`

### 3. 打开示例

```bash
./scripts/run_demo.sh 15 --show-route-debug --duration 120
./scripts/run_demo.sh 17 --show-cable-state --duration 120
```

也可以直接运行：

```bash
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/25_faive_index_pip_surface_cable.xml \
  --show-route-debug --show-cable-state --duration 120
```

## 精选 Demo

仓库只保留学术项目主页中展示的模型：

| 类别 | Demo | 内容 |
|---|---|---|
| 滑轮和卷轴 | 09、10、11、12、15、21 | 卷轴包络、双滑轮、预绕储备、摩擦与轮轴传动 |
| 滚动/柔顺关节 | 13、14、16、20 | 圆柱、凸 mesh、被动和受控马鞍关节 |
| 张拉整体 | 17、18、19 | 插件/原生基线以及混合刚度与松弛 |
| Faive PIP | 24、25 | 双虚拟铰链基线和自由刚体表面绳索模型 |

准确模型名见[示例目录](docs/DEMO_CATALOG.md)。

## MJCF 接口

编译 MJCF 前必须加载动态库：

```python
from pathlib import Path
import mujoco

mujoco.mj_loadPluginLibrary(str(Path("libcable_unilateral.dylib").resolve()))
model = mujoco.MjModel.from_xml_path("model.xml")
```

最小原生路线配置：

```xml
<extension>
  <plugin plugin="mujoco.cable.unilateral">
    <instance name="cable">
      <config key="route_mode" value="native"/>
      <config key="home_length" value="auto_initial"/>
      <config key="stiffness" value="1200"/>
      <config key="damping" value="2"/>
      <config key="slack" value="0.0002"/>
      <config key="max_tension" value="80"/>
      <config key="ctrl_mode" value="target_contraction"/>
    </instance>
  </plugin>
</extension>
```

表面路线使用不施加物理力的 seed tendon。`site user` 第一个槽位含义为：

- `user="1"`：真实固定端点；
- `user="2"`：初始化路线 hint，运行时不作为受力节点；
- `user="3"`：运行时仍必须经过的真实 guide；
- `user="0"`：没有插件角色。

```xml
<size nuser_site="1"/>
<tendon>
  <spatial name="route_seed" width="0.000000001">
    <site site="start"/><site site="hint_a"/>
    <site site="hint_b"/><site site="end"/>
  </spatial>
</tendon>

<config key="route_mode" value="surface"/>
<config key="mesh_route_mode" value="guided_surface"/>
<config key="route_tendon" value="route_seed"/>
<config key="wrap_geoms" value="surface_a surface_b"/>
```

`wrap_geoms` 必须和 role-2 hint 一一对应。表面模式 actuator 必须使用 `gear="0"`，
因为插件会沿求解出的真实路线直接施加刚体力。

12维插件传感器依次输出：

```text
length, velocity, free_length, contraction, extension, tension,
taut, saturated, route_status, tangent_residual,
surface_residual, solver_iterations
```

## 验证

```bash
export CABLE_PLUGIN_LIBRARY="$PWD/build/plugin/libcable_unilateral.dylib"  # Linux 使用 .so
python -m unittest discover tests
python scripts/smoke_cpp_plugin.py \
  --plugin "$CABLE_PLUGIN_LIBRARY" \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml
```

测试覆盖单边本构、圆柱几何、多表面路线、凸/非凸 mesh、滑轮、马鞍关节、张拉整体、
Faive PIP 对照以及仓库相对路径检查。

## 打包 Release

```bash
./scripts/package_release.sh build/plugin/libcable_unilateral.dylib dist
```

推送 `v0.1.1` 一类 tag 后，GitHub Actions 会构建各平台压缩包、生成 SHA-256 文件并
上传到 GitHub Release。

## 当前限制

- 无绳质量、下垂、弯扭、波传播和绳索自接触；
- mesh 路线保持初始化选择的同伦类/走廊，运行时不会自动全局换侧；
- 非凸包络要求障碍 mesh 闭合、法向一致且无自交；
- 滚动关节示例尚未实现完整的绳面无滑移速度约束；
- 当前 Faive PIP 回归中，离散双表面桥接路线保持无穿透，但界面切向角最坏约为
  `21.2 deg`。Demo 25 应视为研究对照模型，而不是经过实物验证的数字孪生；
- 二进制 Release 必须匹配操作系统、CPU 架构和 MuJoCo ABI。

## 许可证

代码采用 Apache License 2.0。资产归属见 [LICENSE](LICENSE) 与
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
