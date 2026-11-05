# 第一个绳驱模型

本教程使用一个定滑轮和自由物块。物块是 `freejoint`，不是预先限制在竖直导轨上的 slide；收绳后它只受到重力、绳力和可能的刚体接触力。

## 1. 下载并放置模型

下载[完整最小 surface 模型](downloads/minimal_surface.xml)，放到仓库根目录。这个模型只用基本几何，不需要外部 STL。

```{literalinclude} downloads/minimal_surface.xml
:language: xml
:lines: 1-24
```

先明确四个对象：`start` 是固定端，`hint` 选择圆柱上方的绕行侧，`end` 固定在物块上，`pulley_wrap` 是真实绳路的障碍面。`hint` 在初始化后不再是绳长或受力节点。

## 2. 手动收绳

```bash
# macOS；Linux 将 mjpython 改为 python、dylib 改为 so。
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model minimal_surface.xml \
  --no-auto-control --show-cable-state --duration 120
```

在右侧 **Control** 面板调整 `pull`：

| 输入 | 含义 |
|---|---|
| `0` | 不额外收绳；仍有模型设定的预紧 |
| `0.010` | 自由长度目标比原来短 10 mm |
| `0.030` | 收绳 30 mm；不是施加 0.03 N，也不是强制物块移动 30 mm |

控制量 `c` 是收缩量，正数使自由长度变短。当前几何长度 `L`、自由长度 `L_f` 和拉力 `T` 是不同的量，物块实际运动由动力学决定。

## 3. 自动演示

```bash
mjpython scripts/view_cpp_plugin_demo.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model minimal_surface.xml \
  --auto-actuator pull --ctrl-final 0.030 \
  --ctrl-ramp-start 0.5 --ctrl-ramp-end 2.5 \
  --show-cable-state --duration 120
```

先关闭自动输入再用鼠标调滑块。否则自动控制器会在下一步覆盖你的值，看起来就像滑块“失效”。

## 4. 为什么 gear 是零

```xml
<actuator>
  <plugin name="pull" instance="rope" tendon="seed"
          gear="0" ctrllimited="true" ctrlrange="-0.02 0.06"/>
</actuator>
<sensor><plugin name="rope_state" instance="rope"/></sensor>
```

surface 模式由插件直接将表面节点的绳力写入刚体广义力。如果 transmission 再沿原生 seed tendon 施力，会重复加载。因此必须 `gear="0"`。它不表示“没有驱动”：控制仍进入插件，受力经 passive 阶段施加。

## 5. 改成被动绳或 native

- [最小被动模型](downloads/minimal_passive.xml)：不含 actuator；用鼠标扰动物块，观察伸长受拉与恢复。刚度有限，不是刚性长度约束。
- [最小 native 模型](downloads/minimal_native.xml)：使用原生两点路线和插件单边本构，actuator 使用正常 tendon transmission，不要照搬 surface 的零 gear。

这三份 XML 都经过加载和短时动力学检查。接下来阅读 [MJCF 结构](modeling.md) 和 [驱动](actuation.md)。
