# MuJoCo 接口速查：绳驱与并联机器人

**来源与版本。** 本页依据 [MuJoCo 3.4.0 官方文档（Google DeepMind）](https://mujoco.readthedocs.io/en/3.4.0/overview.html) 整理，按机器人建模任务重组，示例为 MuJoCable 编写。下文的原生接口以 **3.4.0** 为准；使用其他版本时，请切换到[官方文档](https://mujoco.readthedocs.io/en/stable/)的对应版本。MuJoCable 专用配置单独标注。

## 按机构选接口

| 任务 | 首选接口 | 建模边界 |
|---|---|---|
| 串联臂、欠驱动手指 | `body/joint`；对需要驱动的关节、卷轴或 tendon 配置 `actuator` | 被动关节不加电机；在 MJCF 树上给出真实轴线和惯量。 |
| 四杆、Delta、Stewart、平行夹爪 | 用 `body/joint` 建开链，用 `equality/connect` 或 `equality/weld` 闭合；用 `equality/joint` 表示齿轮比例等标量耦合 | 实体闭链与绳索闭合不是一回事。理想比例约束不等于齿面接触和摩擦。 |
| 缆索并联机器人（CDPR）、张拉结构 | 平台用 `freejoint` 或所需的平面关节；每条索分别建 `tendon/spatial` 或 MuJoCable 实例 | 张力只能拉。不要用每条绳的双边等式替代松弛；检查张力可行性、预紧和失张后的运动。 |
| 简单绳路、滑轮组基线 | 原生 `tendon/spatial` 按顺序使用 `site`、球/圆柱 `geom`；`pulley` 可定义分支长度倍率 | 原生 `pulley` 是长度分支，不是有惯量、轴承和可转动绳槽的刚体。 |
| 绕不规则面、分段摩擦、卷扬放绳 | MuJoCable `surface` 路由；用 body/hinge 表示物理滑轮或卷轴 | 只计算指定的包络体和导向；不自动获得自由悬垂、孔壁三维接触或任意拓扑切换。 |
| 滚动关节、孔口和柔性体 | 实体接触用 `geom`、组合代理或 `flex`；绳路另用包络面、真实 guide 或孔口模型 | 碰撞面、可见 mesh 和绳索包络面可不同。不能从 STL 上存在孔推断原生 mesh 碰撞会保留孔。 |

## MJCF 格式、单位与坐标

| 层级 | 常用元素或属性 | 约定 |
|---|---|---|
| 模型与资源 | `<mujoco>`、`compiler`、`option`、`size`、`default/class`、`include`、`asset/mesh`、`asset/material` | 用相对文件路径；核对 `meshdir`、`scale` 和 CAD 原点。`default` 统一复用属性，`childclass` 可传给子级。 |
| 刚体与参考点 | `worldbody/body`、`inertial`、`geom`、`site`、`camera` | `body pos` 相对父坐标系，其子元素用所属刚体的局部坐标；`site` 自身不是碰撞体。需要精确质量和质心时显式设 `inertial`。 |
| 关节 | `hinge`、`slide`、`ball`、`freejoint`；`axis`、`pos`、`limited/range`、`ref`、`stiffness/springref`、`damping`、`armature`、`frictionloss` | `hinge` 和 `slide` 是标量自由度；空间自由刚体的 `freejoint` 放在 `worldbody` 的直接子刚体上。不要把 `armature` 当作轴承阻尼。 |
| 交叉引用 | `tendon`、`equality`、`contact`、`actuator`、`sensor`、`keyframe/key` | 用有语义的名称引用对象，例如 `thumb_tip_anchor`、`winch_hinge`。`key` 可记录 `qpos`、`qvel`、`ctrl` 等初始条件。 |

统一采用 m、kg、s、N、N·m，并在教程 XML 中显式设 `<compiler angle="radian"/>`。没有显式设定时，MJCF 的默认角度输入通常为度；运行时的 `qpos` 铰链角和 `qvel` 角速度使用 rad 与 rad/s。`compiler angle` **不负责换算控制输入**。`quat` 顺序为 `w x y z`；可移动平台 `freejoint` 占 7 个 `qpos` 和 6 个 `qvel` 元素，不要逐元素对四元数作平移式积分。

官方参考：[MJCF XML](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html)、[建模与坐标系](https://mujoco.readthedocs.io/en/3.4.0/modeling.html#coordinate-frames)。

## 绳路、卷轴与控制量

| 接口 | 关键字段 | 怎么用 |
|---|---|---|
| 原生 `tendon/spatial` | `site/@site`、`geom/@geom`、`sidesite`、`pulley/@divisor` | 真实 via site 运行时始终在线路上；`sidesite` 是原生包络侧参考，不是 via。3.4.0 原生包络对象是球/圆柱，连续两个包络 geom 之间需要 site。 |
| 原生 `tendon/fixed` | `<joint joint="…" coef="…"/>` | 标量关节坐标的加权和；适合虚拟差动或耦合，不生成三维绕线路径。 |
| 原生张力基线 | `limited`、`range`、`springlength`、`stiffness`、`damping`、`frictionloss` | 只研究几何上限可设 `limited="true" range="0 Lmax"`（将 `Lmax` 换成数值）；在过长侧产生约束反力。普通弹簧和阻尼并不自动等价于含松弛判定的单边绳，`frictionloss` 也不是包角摩擦。 |
| 原生 actuator | `motor`、`position`、`velocity`、`general`、`muscle`；`joint/tendon/site`、`gear`、`ctrlrange`、`forcerange` | `ctrl` 可表示力指令、位置/速度参考或肌肉激励，取决于执行器及传动；`ctrlrange` 限制输入，`forcerange` 限制执行器标量输出。 |
| MuJoCable 绳路 | `route_mode`、`route_tendon`、`wrap_geoms`；`site user` 角色 | `surface` 中端点 `1`、初始化 hint `2`、真实 guide `3` 各有不同作用。`gear="0"` 避免原生 seed 再施一次力，不能用 seed 的原生长度替代插件实际绳长。 |
| MuJoCable 驱动与摩擦 | `target_contraction`、`joint_spool_angle`、`capstan_mu` 等 | 位移输入、卷轴角度、轴承阻尼、绳/轮摩擦是不同物理量。固定半径卷轴可用 $\Delta c=\pm r\,\Delta\theta$ 做符号/单位校验；正负取决于绕向。 |

**最小原生例子。** 下载 <a href="downloads/mujoco_native_cable.xml" download>mujoco_native_cable.xml</a>；圆柱固定、载荷沿 slide 运动。`pull` 是单位增益 motor，`gear="-1"` 使正输入趋向缩短绳路。这个例子施加受控拉力，**不是**带松弛本构的卷扬。

```{literalinclude} downloads/mujoco_native_cable.xml
:language: xml
:start-at: <tendon>
:end-at: </actuator>
```

官方参考：[原生 spatial tendon](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#tendon-spatial)、[执行器计算](https://mujoco.readthedocs.io/en/3.4.0/computation/index.html#actuation-model)。插件特有设置见[绳路](routing.md)、[驱动](actuation.md)和[摩擦](friction.md)。

**并联平台的张力检查。** 对无摩擦且各段等张力的绳，将长度雅可比定义为 $\dot{\boldsymbol L}=J_L\boldsymbol v$。该组绳产生的广义力为 $\boldsymbol\tau=-J_L^\mathsf{T}\boldsymbol T$。检查期望广义力能否由满足 $0\le T_i\le T_{i,\max}$ 的张力实现，而不是只看矩阵秩。直线绳的长度导数可由两端 `mj_jacSite` 和绳方向求得；多表面路径则应基于实际路线，不能用 seed 长度求导来代替。存在分段摩擦时，也不能用一个全绳等张力标量替代分段受力。

## 闭链、接触与柔性

| 目的 | 接口与格式 | 调试重点 |
|---|---|---|
| 两条支链在一点相接 | `equality/connect site1="a" site2="b"`；或 `body1/body2/anchor` 写法 | site 方式约束两点重合，不锁相对姿态。闭链初始位姿应先对齐，避免启动冲击。 |
| 固定两支链相对姿态 | `equality/weld`；`site1/site2` 对齐两坐标系，或用 `body1/body2/relpose` | 与开放树中“同一刚体上无关节地刚接”区分；weld 增加约束，不能用来替代滚动轴承。 |
| 理想比例、定长或联动 | `equality/joint joint1="a" joint2="b" polycoef="0 1 0 0 0"`；`equality/tendon tendon1="a" tendon2="b"` | 标量关节或原生绳长的关系按参考构型偏差解释。等式是双边的；不要约束 MuJoCable 的 seed 来冒充实际绳长约束。 |
| 接触与排除 | `geom contype/conaffinity/condim/friction`；`contact/pair geom1/geom2`；`contact/exclude body1/body2` | 动态配对要核对掩码，还要看同体/父子等过滤；`pair` 可显式指定一对几何与接触参数。`geom friction` 的三项对应滑动、扭转和滚动，第三项不是接触几何的纯滚动约束。 |
| 非凸、孔以及材料柔顺 | 碰撞用复合几何或合适的 `flexcomp/flex`；柔性梁的低阶模型可用 `joint stiffness/damping` | 常规 mesh geom 碰撞使用凸表示，显示的 STL 可非凸。标定等效铰链刚度，或独立验证 flex 的材料参数及网格收敛。 |
| 数值稳定性 | `option timestep/integrator/solver/iterations/tolerance`、约束 `solref/solimp` | 正值 `solref` 形式为时间常数和阻尼比，`solimp` 调整约束阻抗。先查几何与约束秩，再减小步长做收敛对照，不只增加迭代次数。 |

**最小闭链例子。** 下载 <a href="downloads/mujoco_four_bar.xml" download>mujoco_four_bar.xml</a>：用三根树内铰链和一个点闭合构成平面四杆。在非奇异构型下，三个标量关节坐标受两个独立平面闭合方程限制，剩一个有效自由度。调 `crank_target` 控制主动杆，比较 `data.site("coupler_tip").xpos` 与 `data.site("rocker_tip").xpos` 的距离。软约束在载荷下可有残差；不要把 `model.nv` 当成闭链机构独立自由度数。

```{literalinclude} downloads/mujoco_four_bar.xml
:language: xml
:start-at: <equality>
:end-at: </equality>
```

两个模型均不依赖插件与外部资产。在文件所在目录运行（macOS；其他平台用 `python` 替换 `mjpython`）：

```bash
mjpython -m mujoco.viewer --mjcf=mujoco_native_cable.xml
mjpython -m mujoco.viewer --mjcf=mujoco_four_bar.xml
```

官方参考：[Equality](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#equality)、[Contact](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#contact)、[约束参数](https://mujoco.readthedocs.io/en/3.4.0/modeling.html#solver-parameters)。

## Python 与 C 运行时

| 阶段 | API / 数据 | 读写规范 |
|---|---|---|
| 编译和重置 | `MjModel.from_xml_path`（C：`mj_loadXML`）、`MjData(model)`（C：`mj_makeData`）、`mj_resetData`、`mj_resetDataKeyframe` | 插件模型加载前先注册动态库（`mj_loadPluginLibrary`）；重置或手改 `qpos/qvel` 后调用 `mj_forward` 更新派生量。 |
| 信号输入与积分 | `data.ctrl`、`mj_step`、`mj_step1/mj_step2`、`mjcb_control` | 先设输入再步进；依赖当前运动学反馈时可分步计算，但分步接口不能保留 RK4 的积分语义。 |
| 外力与扰动 | `qfrc_applied`、`xfrc_applied`、`mj_applyFT` | 前者按 `nv` 索引，后者按 body；`mj_applyFT` 可把指定世界位置的力和力矩累加到目标广义力数组。按实验约定撤销外力，写入值不会因一次步进自动清零。 |
| 位姿、速度和雅可比 | `mj_name2id`、`jnt_qposadr/jnt_dofadr`、`site_xpos/site_xmat`、`mj_jacSite`、`mj_objectVelocity` | 关节 ID 不等于状态地址；雅可比列数用 `nv`。`mj_objectVelocity` 输出角速度在前、线速度在后，坐标朝向由 `flg_local` 决定。 |
| 接触与闭链 | `ncon`、`contact`、`mj_contactForce`、`eq_active`、`efc_type/efc_id/efc_pos/efc_force` | `mj_contactForce` 返回接触坐标系中的六维力/矩；通过类型与 ID 找约束行，不用固定行号。启停闭合用 `data.eq_active[模型中的等式 ID]`。 |
| 系统分析与复现 | `mj_getState/mj_setState`、`mj_integratePos/mj_differentiatePos`、`mj_inverse`、`mjd_transitionFD`、`mj_mulM` | 复现实验要记录状态位掩码与控制；姿态增量用四元数感知的 API；线性化在绳索松紧或接触切换附近应先验证局部有效性。 |
| 生成模型与交互 | `MjSpec.from_file`、`spec.compile()`、`mujoco.viewer.launch_passive`、`viewer.sync()` | 用 `MjSpec` 或结构化 XML 生成重复支链；改拓扑应重编译。使用被动 viewer 时由脚本推进物理并同步画面。 |

```python
import mujoco

m = mujoco.MjModel.from_xml_path("mujoco_native_cable.xml")
d = mujoco.MjData(m)
d.actuator("pull").ctrl[0] = 0.75
for _ in range(20):
    mujoco.mj_step(m, d)

mujoco.mj_forward(m, d)  # Recompute observations at the final state.
length_m = float(d.sensor("cable_length").data[0])
rate_m_s = float(d.sensor("cable_rate").data[0])
pull_N = float(d.sensor("pull_output").data[0])
print(f"L={length_m:.6f} m, dL/dt={rate_m_s:.6f} m/s, motor={pull_N:.3f} N")
```

官方参考：[Python API 与命名访问](https://mujoco.readthedocs.io/en/3.4.0/python.html#named-access)、[C API Functions](https://mujoco.readthedocs.io/en/3.4.0/APIreference/APIfunctions.html)、[C API Types](https://mujoco.readthedocs.io/en/3.4.0/APIreference/APItypes.html)。

## 记录哪些量

| 要回答的问题 | 记录接口 | 注意事项 |
|---|---|---|
| 电机有没有按预期收绳？ | `ctrl`、卷轴 `jointpos/jointvel`、`actuatorfrc` 或 `data.actuator_force` | 记录输入单位、gear 和输出符号。原生执行器标量输出不等于关节力矩，更不等于所有分段的张力。 |
| 绳长、速度、松弛是否正确？ | 原生 `tendonpos/tendonvel` 或 `data.ten_length/ten_velocity`；插件用 `sensor/plugin` | 插件应记录实际路由长度、自由长度、张力、`taut` 与路线状态；传感器维度从 `sensor_dim` 读，不硬编码索引到整个 `sensordata`。 |
| 闭链是否闭合，有没有打滑或穿模？ | 两个 site 的世界坐标、`efc_pos/efc_force`、`contact`、`mj_contactForce` | 报单位、坐标系和采样阶段。滚动要对比接触点相对切向速度；闭链残差与绳路残差分别记。 |
| 末端位姿、夹持和基座反力如何？ | `framepos/framequat`、`framelinvel/frameangvel`、`touch`、`force/torque` | `touch` 是区域内接触法向力汇总；`force/torque` 是具有父子刚体语义的传感器，不等同于任意一对 geom 的接触力。 |

做实物或基线对照时，先统一几何、质量惯量、初始姿态、输入单位、预紧和采样时间，再逐步切换绳路或摩擦建模方式。并联平台额外记录每根绳张力与任务空间扳手；对可能失张或接近奇异位姿的构型单独检查。日志同时记录 MuJoCo 版本、模型版本、插件版本和 `timestep`，瞬时状态用数组副本保存。

官方参考：[Sensors](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#sensor)、[Simulation](https://mujoco.readthedocs.io/en/3.4.0/programming/simulation.html)。MuJoCable 的字段与回调见[传感器](sensors.md)和[接口与架构](architecture.md)；插件通用规范见 MuJoCo 官方 [Engine plugins](https://mujoco.readthedocs.io/en/3.4.0/programming/extension.html#engine-plugins)。
