# 全部本地 Demo 运行命令

本文档中的命令均从仓库根目录执行，并统一使用当前源码编译出的最新版插件。

## 1. 构建与环境

```bash
python -m pip install -e .
MUJOCO_DIR="$(python -c 'import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)')"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_PYTHON_PACKAGE_DIR="$MUJOCO_DIR"
cmake --build build --config Release

# macOS
export MUJOCABLE_PLUGIN="$PWD/build/plugin/libcable_unilateral.dylib"
# Linux 请改为：
# export MUJOCABLE_PLUGIN="$PWD/build/plugin/libcable_unilateral.so"
```

`scripts/run_demo.sh` 在 macOS 自动调用 `mjpython`，在 Linux 调用 `python3`。
使用已解压的二进制 Release 时，把 `MUJOCABLE_PLUGIN` 指向 `lib/` 中的动态库即可。

## 2. 滑轮、卷轴与孔口

```bash
./scripts/run_demo.sh 09 --show-route-debug --duration 120
./scripts/run_demo.sh 10 --show-route-debug --duration 120
./scripts/run_demo.sh 11 --show-route-debug --duration 120
./scripts/run_demo.sh 12 --show-route-debug --duration 120
./scripts/run_demo.sh 15 --show-route-debug --duration 120
./scripts/run_demo.sh 21 --show-route-debug --duration 120
./scripts/run_demo.sh 29 --show-route-debug --duration 120
./scripts/run_demo.sh 31 --duration 120
./scripts/run_demo.sh 32 --duration 120
```

- 09：卷筒、固定滑轮和自由重物。
- 10：多圆柱连续包络与公切线。
- 11：有符号卷绕余量与反转放绳。
- 12：Capstan 分段张力。
- 15：最小单圆柱 surface route。
- 21：轮轴机构的力/位移放大。
- 29：有转动惯量的自由滑轮、速度定向摩擦和能量耗散。
- 31：普通 mesh 凸化与 rigid-flex 真孔碰撞对照。
- 32：role-3 理想孔口及局部 Capstan 摩擦。

## 3. 滚动关节、机械手与张拉整体

```bash
./scripts/run_demo.sh 13 --show-route-debug --duration 120
./scripts/run_demo.sh 14 --show-route-debug --duration 120
./scripts/run_demo.sh 16 --show-route-debug --duration 120
./scripts/run_demo.sh 20 --show-route-debug --duration 120
./scripts/run_demo.sh 17 --duration 120
./scripts/run_demo.sh 18 --duration 120
./scripts/run_demo.sh 19 --duration 120
./scripts/run_demo.sh 24 --duration 120
./scripts/run_demo.sh 25 --show-route-debug --duration 120
./scripts/run_demo.sh 26 --show-route-debug --duration 120
./scripts/run_demo.sh 27 --show-route-debug --duration 120
./scripts/run_demo.sh 28 --show-route-debug --duration 120
```

Demo 18 和 24 是原生 tendon/虚拟铰链基线；其余命令均由最新版插件加载。
Demo 16 是无 actuator 的被动韧带关节，可在 viewer 中用鼠标施加外力观察回弹。

## 4. 对数螺旋机器人：双余量、双侧放绳与自碰撞

自动完成“两侧同步收余量，再进行差动收放”：

```bash
./scripts/run_demo.sh 33 \
  --mode differential --reserve 0.025 --contraction 0.050 \
  --ramp-time 4 --period 12 --duration 120
```

手动版本：先将两个控制量同时调到 `0.025 m`，再保持二者之和为 `0.050 m`
进行差动控制。

```bash
./scripts/run_demo.sh 33 --mode manual --duration 120
```

五个孔口摩擦版本位于
`cable_plugin_demos/open_spirob_friction_variants/`。例如打开 `mu=0.10`：

```bash
mjpython scripts/view_log_spiral_dual_reserve.py \
  --plugin "$MUJOCABLE_PLUGIN" \
  --model cable_plugin_demos/open_spirob_friction_variants/open_spirob_mu_0p100.xml \
  --mode differential --reserve 0.025 --contraction 0.050 \
  --ramp-time 4 --period 12 --duration 120
```

可将文件名依次替换为 `0p000`、`0p015`、`0p050`、`0p100` 和 `0p200`。

## 5. 物理边界 Demo

Demo 30 是有质量连续绳的独立验证器，不代表当前无质量插件已支持材料输运：

```bash
./scripts/run_demo.sh 30 --case sliding --duration 120
./scripts/run_demo.sh 30 --case stick --duration 120
```

## 6. 无界面验证

```bash
python scripts/validate_release_tree.py
CABLE_PLUGIN_LIBRARY="$MUJOCABLE_PLUGIN" PYTHONPATH=python \
  python -m unittest discover tests
python scripts/smoke_cpp_plugin.py \
  --plugin "$MUJOCABLE_PLUGIN" \
  --model cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml
python scripts/analyze_free_rotating_pulley.py \
  --plugin "$MUJOCABLE_PLUGIN" --strict
python scripts/check_log_spiral_dual_reserve.py \
  --plugin "$MUJOCABLE_PLUGIN"
```
