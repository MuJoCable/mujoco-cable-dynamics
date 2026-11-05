# 传感器与 Python

## 配置 sensor

```xml
<sensor>
  <plugin name="rope_state" instance="rope"/>
</sensor>
```

native 输出 8 维，surface 输出 12 维。没有配置 sensor，并不代表插件没有计算受力；但你无法仅通过 `sensordata` 读取这个实例的状态。

| 索引 | 名称 | 单位 | 解释 |
|---:|---|---|---|
| 0 | length | m | 实时求解路线长度 |
| 1 | velocity | m/s | 几何绳长变化率 |
| 2 | free_length | m | 本构自由长度，不含 slack 死区 |
| 3 | contraction | m | 控制映射后的实际收缩量，可能被平滑/积分 |
| 4 | extension | m | `length - free_length - slack` |
| 5 | tension | N | 源端本构张力；有摩擦时不是每段张力 |
| 6 | taut | 0/1 | 正张力状态 |
| 7 | saturated | 0/1 | 源端张力达到上限 |
| 8 | route_status | 枚举 | 0 valid，1 degraded，2 invalid |
| 9 | tangent_residual | 求解器度量 | 当前活动路径的切向/优化残差 |
| 10 | surface_residual | m | 表面与穿透相关的几何残差 |
| 11 | solver_iterations | 次 | 当步求解迭代数，缓存命中后可降低 |

`tangent_residual` 不是统一的“角度误差，单位 deg”。不同路径后端的残差应结合实现解释，不能不加区分地与表面距离相比较。

## 按名称读取，不要猜偏移

```python
import numpy as np
import mujoco

sid = model.sensor("rope_state").id
adr = int(model.sensor_adr[sid])
dim = int(model.sensor_dim[sid])
values = np.asarray(data.sensordata[adr:adr + dim]).copy()
length_m, length_rate, free_m = values[:3]
contraction_m, extension_m, tension_N = values[3:6]
if dim == 12 and values[8] > 1:
    raise RuntimeError("invalid cable route; inspect geometry before continuing")
```

`.copy()` 使记录不被下一步仿真覆盖。读取之前至少 `mj_forward` 一次；动力学记录通常在 `mj_step` 后采样。

## 记录 CSV

```python
import csv

with open("cable.csv", "w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["time_s", "length_m", "free_length_m", "contraction_m", "tension_N"])
    for step in range(5000):
        mujoco.mj_step(model, data)
        x = data.sensordata[adr:adr + dim]
        if step % 20 == 0:
            writer.writerow([data.time, x[0], x[2], x[3], x[5]])
```

同时保存模型版本、动态库 SHA-256、步长、控制器、摩擦系数、余量和单位，才方便复现。论文案例的 `run.py` 会将新输出与历史文件分开保存。

## 在 viewer 中看长度与松弛

`--show-cable-state` 显示绳长、有效参考长度、应变和张力等标签。默认的松弛配色可使零张力绳变灰；需要保留原始拓扑颜色时使用 viewer 支持的 `--no-slack-color`。先运行 `--help` 确认当前脚本选项，不要把私有 OPH 启动器选项与通用 viewer 混用。

surface 模式的 `actuator_force` 按负拉力约定报告 `-T`，实际施力在 `qfrc_passive`。因此不能只看 `qfrc_actuator` 就断言“绳没有驱动”。native 的主要 transmission 则由 MuJoCo actuator 通道映射，另有插件修正项。

当前 12 维 sensor 没有输出完整接触点数组或所有分段张力数组。若研究每段载荷，应使用案例分析脚本、已知传播关系或新增诊断输出，不要把第 5 项复制到每个绳段。

## 处理 invalid

invalid 步会禁用该步绳力，部分本构输出归零。此时 length=0 **不代表真实绳瞬间缩成零长**。先查 route_status 和警告，不要把该步当作正常零张力样本做平均。传感阶段不会偷偷重算一条有效路线来掩盖刚才的力学失败。
