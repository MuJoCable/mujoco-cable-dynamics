# 安装与版本

## 推荐：从公开源码构建

所有命令在仓库根目录运行。新建 Python 环境，避免把已有旧动态库误当作最新版。

```bash
git clone https://github.com/MuJoCable/mujoco-cable-dynamics.git
cd mujoco-cable-dynamics
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "mujoco==3.4.0" numpy cmake pillow matplotlib

MUJOCO_DIR="$(python -c 'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release
```

这个流程直接构建 C++ 动态库，不依赖仓库的 editable Python 打包。当前公开提交的 `pip install -e .` 可能因 setuptools 的顶层目录发现而失败；不要把这个打包问题误判成插件编译失败。需要运行 Python 辅助模块的测试时使用：

```bash
PYTHONPATH=python python -m unittest discover tests
```

如果动态库不在默认 `build/plugin/`，先设置 `CABLE_PLUGIN_LIBRARY` 指向该库。测试必须真正找到库；跳过测试不代表验证通过。

需要 C++17 编译器和 CMake。macOS 可使用已安装的 Xcode Command Line Tools；Linux 可使用系统 C++ 编译工具链。不要把不同 MuJoCo 版本、CPU 架构或操作系统编译的动态库混用。

| 平台 | 典型动态库 | GUI 入口 |
|---|---|---|
| macOS | `build/plugin/libcable_unilateral.dylib` | `mjpython` |
| Linux | `build/plugin/libcable_unilateral.so` | `python` |

本手册的构建与案例验证基线是 **MuJoCo 3.4.0**。这不是对所有未来 MuJoCo 版本、Windows 或 GPU 后端的兼容承诺。

Windows 用户可使用独立的[Windows 安装与肘关节教程](https://mujocable.github.io/tutorials/windows/zh.html)（[English](https://mujocable.github.io/tutorials/windows/)），按该发行包的版本和架构说明操作；不要直接复制本页 macOS 的 `.dylib` 命令。

## 检查是否加载成功

```bash
python scripts/smoke_cpp_plugin.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml \
  --steps 50
```

Linux 把 `.dylib` 改为 `.so`。先完成这个无界面检查，再排查 OpenGL/viewer 问题。

## 打开已有模型

```bash
# 启动器按平台选择 mjpython 或 Python。
./scripts/run_demo.sh 15 --show-cable-state --duration 120

# 手工控制，关闭自动输入。
./scripts/run_demo.sh 15 --no-auto-control --show-cable-state --duration 120

# 论文案例入口。
python examples/paper/run.py list
python examples/paper/run.py view rolling_mesh --show-cable-state
```

此通用 viewer 的 `--duration` 是仿真循环重置时长，不是保证按墙钟时间退出。慢模型推进 1 秒仿真时间可能需要更长现实时间。

## 二进制 Release

可在 [Releases](https://github.com/MuJoCable/mujoco-cable-dynamics/releases) 查看已发布的平台包。**源码更新、已合并 PR 与已发布二进制不是同一件事。** 为使用本手册描述的最新接口，优先从当前源码构建；不要假定旧 Release 已包含全部更新。

下载的 XML 通常还需要相对路径下的 mesh/纹理。使用案例页的“模型与资产 ZIP”，或 clone 完整仓库；只下载单个 XML 不一定能加载。

## 更新动态库后

退出旧 viewer，重新编译并新开进程。XML reload 不保证替换已由进程加载的动态库。当前每实例状态为 8 个标量；旧版 5 状态的 checkpoint 或 MJB 不应直接复用。保留 MJCF 作为可迁移源文件。
