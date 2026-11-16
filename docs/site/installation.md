# 安装与运行

本页适用于 macOS 和 Linux，使用 Python 3.12、MuJoCo 3.4.0、Git 和 C++17 编译器。Windows 请按[中文安装教程](https://mujocable.github.io/tutorials/windows/zh.html)操作。

## 1. 下载仓库

```bash
git clone https://github.com/MuJoCable/mujoco-cable-dynamics.git
cd mujoco-cable-dynamics
```

后续命令均在该目录执行。

## 2. 创建环境并安装依赖

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "mujoco==3.4.0" numpy cmake pillow matplotlib
```

## 3. 编译插件

```bash
MUJOCO_DIR="$(python -c 'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release
```

编译产物：

| 平台 | 插件文件 |
|---|---|
| macOS | `build/plugin/libcable_unilateral.dylib` |
| Linux | `build/plugin/libcable_unilateral.so` |

## 4. 检查插件

设置插件位置，运行 50 步无界面测试：

```bash
export CABLE_PLUGIN_LIBRARY="$(python -c 'from pathlib import Path; import sys; print(Path("build/plugin/libcable_unilateral" + (".dylib" if sys.platform == "darwin" else ".so")).resolve())')"
python scripts/smoke_cpp_plugin.py \
  --plugin "$CABLE_PLUGIN_LIBRARY" \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml \
  --steps 50
```

## 5. 打开 viewer 并手动收绳

```bash
./scripts/run_demo.sh 15 --no-auto-control --show-cable-state --duration 120
```

展开右侧 **Control**，调整 `cable_contraction`。输入 `0.010` 表示收绳 10 mm。关闭窗口后返回终端。

## 6. 运行自动演示

```bash
./scripts/run_demo.sh 15 \
  --auto-actuator cable_contraction --ctrl-final 0.030 \
  --show-cable-state --duration 120
```

自动演示期间由脚本更新控制量；需要自己拖动滑块时，使用第 5 步命令。

## 7. 打开其他案例

```bash
python examples/paper/run.py list
python examples/paper/run.py view rolling_mesh --show-cable-state
python examples/paper/run.py view finger --show-cable-state
python examples/paper/run.py view spirobs --show-cable-state
```

这些命令默认手动控制。截图、视频、参数和完整模型包见[案例库](gallery.md)。自己的第一个模型见[快速入门](quickstart.md)。

## 8. 运行测试（可选）

```bash
PYTHONPATH=python python -m unittest discover tests
```

## 下次打开与更新

重新打开终端后，进入仓库目录并运行：

```bash
source .venv/bin/activate
./scripts/run_demo.sh 15 --no-auto-control --show-cable-state
```

更新代码后，关闭原 viewer，重新执行第 3 步编译，再打开模型。预编译包见 [Releases](https://github.com/MuJoCable/mujoco-cable-dynamics/releases)。
