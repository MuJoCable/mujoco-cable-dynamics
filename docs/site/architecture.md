# MuJoCo 接口与架构

## 动力学边界

| MuJoCo 内核 | MuJoCable 独立插件 |
|---|---|
| MJCF 编译、body/joint、质量与惯量 | 读取编译后的 site/geom/mesh 信息 |
| 刚体碰撞、摩擦接触、equality、约束 | 在已配置障碍上的中心线绳路 |
| 运动学与时间积分 | 单边轴向本构、收绳状态、分段传力 |
| `mjData`、GUI 与 scene | plugin 实例状态、sensor、visualize 回调 |

绳力可影响刚体接触结果，但两个求解器不是同一个接触模块。`geom friction` 不会直接成为绳摩擦系数，绳的初始化 hint 也不会变成 MuJoCo 接触点。

```{image} /_static/figures/runtime_architecture.svg
:alt: MuJoCo 与 MuJoCable 的数据和力接口图
:width: 100%
```

## 回调与数据流

| 接口 | 做什么 | 何时最容易出错 |
|---|---|---|
| 注册 `mujoco.cable.unilateral` | 声明 actuator、passive、sensor 能力与配置键 | 用旧库加载含新 config 的 XML |
| init / Create | 校验名称、角色、表面与实例 | hint/geom 数量不符、mesh 不合法、surface 非零 gear |
| passive compute | surface 路由、本构、分段力，累加 `qfrc_passive` | invalid 时禁用该步力，不可忽略状态 |
| actuator compute | native 的输出力；surface 主要报告 `-T` | 错把 surface 的零 transmission gear 当成无驱动 |
| advance | 积分/平滑收缩与更新相关状态 | 重置后不重新初始化，或误读一步的速率估计 |
| sensor compute | native 8 维 / surface 12 维 | 把源端 T 误当每段张力 |
| visualize | 在 MuJoCo scene 绘制实时路径 | 把 seed 折线当作真实路径 |
| reset / copy | 管理标量状态、局部走廊与缓存 | 沿用旧状态布局；共享可写路线状态 |

MuJoCable 不要求 Python overlay 才能绘制真实 surface 绳路。项目 Python viewer 另外添加标签、调试标记和 native 路径显示等辅助功能。

## C++ 模块

- `UnilateralCable`：配置、本构、控制状态、摩擦、施力与回调入口。
- `NativeTendonRoute`：读取 MuJoCo 已计算的原生 tendon 路径信息。
- `SurfaceEnvelopeRoute`：圆柱/mesh 路线、接触点、方向、状态与残差。

这些类是当前实现内部模块，不是已经承诺 ABI 稳定的外部 SDK。用户主要通过 MJCF 和 MuJoCo 的 C/Python 接口使用。

## 为什么 native 与 surface 的施力不同

native actuator 的 `-T` 通过 MuJoCo tendon transmission 映射到广义力，插件可额外加入相应修正。surface 路径与原生 seed 不同，因此统一从 runtime 节点向刚体施力，actuator transmission 必须置零以避免双重作用。

## 独立插件与内核扩展

默认 explicit 路径、传力、sensor 和可视化可作为独立动态库使用。当前 CPU 插件有状态、有缓存、存在迭代求解；不因放入插件目录就自动支持 MJX/Warp/GPU。

源码里的 `implicit_compliant` 依赖额外的 `mjpPluginImplicit` 等宿主接口和编译条件。标准 MuJoCo 3.4.0 下不能把这个实验配置当作已启用的隐式求解。验证、发布与文档应分别记录“独立插件能力”和“需要修改宿主的扩展”。

MuJoCo 通用插件机制参考[官方 Extensions 文档](https://mujoco.readthedocs.io/en/stable/programming/extension.html)。这份手册描述 MuJoCable 的实际实现，不替代 MuJoCo 官方 API 参考。
