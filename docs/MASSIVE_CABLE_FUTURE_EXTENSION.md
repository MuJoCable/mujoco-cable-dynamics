# 有质量绳索后续扩展记录

> 状态：设计归档，暂缓实现。当前发布版 MuJoCable 仍采用无质量单边绳模型。

## 1. 目标与边界

后续扩展的目标是在保留现有圆柱、凸 mesh、非凸 mesh 和多表面连续包络能力的基础上，增加绳材料沿路径运输时的质量、惯性、分布重力和摩擦动力学。

现有插件继续负责：

- 端点、guide 和初始化 hint 的 XML 语义；
- 圆柱切线/测地线与不规则 mesh 表面包络；
- 多表面公切段、路线拓扑连续性和无穿透检查；
- 路径长度、切向、曲率、包角、接触点和刚体速度；
- 无质量单边弹性绳、卷扬、Capstan 近似和可视化。

有质量后端需要新增：

- 线密度 `lambda` 和材料运输状态；
- 绳索分布重力、切向惯性和向心加速度；
- 绳材料进入或离开活动路径时的质量与动量通量；
- 表面相对速度、静摩擦粘着和动摩擦滑移；
- 绳索惯性与 MuJoCo 刚体自由度的双向耦合。

该扩展不等同于完整可变形绳。首版不计划表示下垂、横波、弯曲、扭转或绳索自碰撞。

## 2. 建议架构

路线几何与绳索动力学保持解耦：

```text
SurfaceEnvelopeRoute
  -> centerline, tangent, curvature, contact body, surface velocity
  -> MasslessUnilateralCable       (现有默认后端)
  -> MassiveTransportCable         (计划中的降阶后端)
  -> DistributedMassiveCable       (远期分布式后端)
```

### 2.1 降阶材料运输模式

对于路径形状由刚体和表面包络决定、绳材料只沿路径运动的问题，每根绳增加材料位移 `s` 和材料速度 `u=ds/dt`。连续体动量平衡写为

```text
lambda Dv/Dt = d(T t)/dl + lambda g + f_contact,
```

其中 `t` 是最终包络路径的单位切向，`T` 是张力，`f_contact` 是单位长度接触力。切向投影决定材料加速度和张力变化，法向投影包含路径曲率产生的向心载荷。

摩擦建议采用静动分离模型：

```text
|f_t| <= mu_static N                    (stick),
f_t = -mu_kinetic N sign(v_relative)    (slip).
```

这比当前固定方向或正则化 Capstan 传播多出粘着历史和材料速度状态。具体 XML 名称仅作为草案，不构成当前 API：

```xml
<config key="mass_model" value="transport"/>
<config key="linear_density" value="0.25"/>
<config key="static_friction" value="0.30"/>
<config key="kinetic_friction" value="0.25"/>
<config key="initial_material_speed" value="0"/>
```

该模式适合固定形状绳路、卷扬、自由滑轮、孔口和连续 mesh 包络，但不表示离开支撑后的下垂绳形。

### 2.2 分布式有质量绳

若需要下垂、波动或局部伸长，应沿材料坐标离散多个质量和应变状态，并解决接触活动集、材料点跨三角面运输及隐式耦合。这是独立动力学后端，可共享 `SurfaceEnvelopeRoute`、BVH 和可视化代码，但不应塞入现有标量本构模型。

## 3. 2018 年决赛理论第 3 题基准

Demo 30 保留第 35 届全国中学生物理竞赛决赛理论第 3 题作为未来有质量后端的首个验收题。该题没有独立悬挂重物：有线密度的绳从右侧地面绳堆被提起，绕过转动滑轮，再沉积到左侧绳堆；两段竖直活动绳的长度均保持为 `L`。

令 `E=exp(mu*pi)`，官方滑动阶段可写为

```text
A - E v^2 = B dv/dt,
A = L g (E - 1) + R g [2 mu/(1 + mu^2)] (E + 1),
B = L (E + 1) + (R/mu) (E - 1).
```

因此

```text
v_s = sqrt(A/E),
v(t) = v_s tanh(t/tau),
tau = B/sqrt(AE),
v_max = min(R omega, v_s).
```

题目中的 `lambda v^2` 地面拾绳动量通量、包络段切向惯性、向心载荷和分布重力都是现有无质量后端缺失的必要项。

当前 Demo 30 通过 `qfrc_applied` 将官方降阶方程施加到独立运输自由度，只验证 MuJoCo 数值积分，不使用 MuJoCable 绳索动力学。未来 `mass_model=transport` 必须在不调用该外部官方加速度函数的情况下重现同样结果。

当前数值基线：

| 验证项 | 官方/解析值 | 当前 MuJoCo 方程积分 |
|---|---:|---:|
| 持续滑动最大速度 | `1.7144867 m/s` | `1.7144727 m/s`（3 s） |
| 滑动转粘着最大速度 | `1.1200000 m/s` | `1.1200000 m/s` |
| 滑动转粘着时刻 | `0.3777383 s` | `0.3778000 s` |
| 最大瞬态速度误差 | - | `1.30e-4 m/s` |
| 运动方程最大残差 | `0` | `4.44e-16` |
| 包络张力关系最大残差 | `0 N` | `1.11e-15 N` |

相关文件：

- `cable_plugin_demos/30_cpho_2018_problem3_massive_rope.xml`
- `scripts/analyze_cpho_2018_problem3.py`
- `scripts/view_cpho_2018_problem3.py`
- `tests/test_cpho_2018_problem3.py`
- `docs/DEMO_30_CPHO_2018_MASSIVE_ROPE.md`

## 4. 恢复开发时的验证顺序

1. `lambda -> 0` 时退化到现有无质量绳结果。
2. 无摩擦直线运输满足动量和能量守恒。
3. 准静态极限恢复 Capstan 张力比。
4. Demo 29 自由滑轮中同时闭合绳速、滑轮角速度、张力差、转矩和能量耗散。
5. Demo 30 在不注入官方加速度的情况下恢复两个解析分支。
6. 圆柱和不规则 mesh 上的材料速度连续，接触点无穿透且无非物理吸力。
7. 对时间步、路径离散数、mesh 分辨率和绳索数量做收敛与性能扫描。
8. 最后再扩展孔口摩擦、移动 mesh 和多表面材料运输。

## 5. 暂缓决定

在现阶段不修改插件 XML API、内部状态数量或 MuJoCo 接口，不把 Demo 30 列为插件功能演示。后续开发应先实现降阶 `MassiveTransportCable`，通过上述解析基准后，再决定是否投入分布式有质量绳后端。
