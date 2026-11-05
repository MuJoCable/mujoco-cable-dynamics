# 如何驱动与收放绳

## 先区分三种量

几何长度 $L(q)$ 由姿态和绳路决定；自由长度 $L_f$ 是绳未受轴向伸长时的可用长度；收缩量 $c$ 是控制映射的输出。正 c 收绳，负 c 放绳，但是否允许负值取决于控制模式与 `ctrlrange`。

$$L_f=L_0-c-p,\qquad e=L-L_f-s.$$

p 是 `pretension_offset`，s 是 `slack`。增加 c 通常增加张力；机器人随后运动，使 L 变化。**控制绳长不是直接控制关节角，更不是给刚体直接指定位置。**

## 控制模式

| `ctrl_mode` | `data.ctrl` 单位 | 映射 | 适合场景 |
|---|---|---|---|
| `target_contraction` | m | 目标收缩量 | 最简单的收放绳、拮抗索 |
| `target_spool_angle` | rad | 角度乘有效半径，并结合储备设置 | 舵机/卷线盘角度命令 |
| `spool_velocity` | m/s | 逐步积分收缩量 | 恒线速度卷扬 |
| `spool_angular_velocity` | rad/s | 乘半径后逐步积分 | 给定卷轴转速 |
| `joint_spool_angle` | 来自物理 hinge 的 rad | 读取 `spool_joint` 相对初始角 | 带惯量/轴承的真实卷轴模型 |

速度模式的累计收缩量在代码中限制为非负；不要假定它与允许有符号放绳的目标模式完全等价。`target_spool_angle` 不会凭空产生一个有质量的卷轴；需要物理卷轴时，自己建 body/hinge，并使用 `joint_spool_angle`。

## 直接控制

```python
import mujoco

mujoco.mj_loadPluginLibrary("build/plugin/libcable_unilateral.dylib")
model = mujoco.MjModel.from_xml_path("minimal_surface.xml")
data = mujoco.MjData(model)
actuator = model.actuator("pull").id
for step in range(3000):
    data.ctrl[actuator] = min(0.030, data.time * 0.010)
    mujoco.mj_step(model, data)
```

每次调用 `mj_step` 前更新 `data.ctrl`。不要在真实绳驱实验中同时直接改 `qpos` 来“帮助弯曲”；那是运动学回放，不是同一种驱动验证。

## 拮抗绳与余量

简单差动为 $c_+=R+u$、$c_-=R-u$。R 用于先收走初始储备余绳，u 控制左右差动。两边路径的增长未必严格相等，所以这个公式并不保证整个运动中两侧都绷紧。

若自由长度含预留量 R，可用 `pretension_offset="-R"` 表示。随后先用共同输入收走余量，再差动控制。`slack` 是本构的死区，不应与预留的 90 mm 可放绳储备混为一谈。

保持两侧绷紧时，释放侧也应跟踪当前路径长度。若希望一侧保持小预张力 $T_p$，忽略平滑与阻尼时可用

$$c_{\rm hold}\approx L_0-p+s-L(q)+T_p/k.$$

这是反馈控制的起点，不是精确的多摩擦段逆模型。必须检查实时 T、饱和、控制范围及两侧耦合；T=0 只能说明未推压，不能说明绳在绷紧状态。案例中的完整控制与 20 mm 卷线盘 CSV 见[标定 SpiRobs](./_generated/calibrated.md)。

## 共享一个控制输入

```xml
<config key="control_source_actuator" value="mcp_differential_drive"/>
<config key="control_scale" value="-1"/>
```

第二根绳读取同名 actuator 的命令，再乘缩放系数；例如 $c_1=u,c_4=-u$。这减少控制通道数，但**不是自动满足功率守恒的机械差速器模型**。要研究电机负载与分配器动力学，应显式建卷轴/传动机构。

`control_scale` 作用于直接目标/目标角命令；不要把它当作速度积分模式或物理 spool joint 的通用齿轮比。

## 物理卷轴

使用 `joint_spool_angle`，设置 `spool_joint`、`spool_radius`，再给卷轴自己的 hinge 配置 motor 或 position actuator。绳实例可不直接接受独立目标收缩输入。是否向轴添加反力矩由 `spool_reaction_torque` 控制，不能在已经包含相同反力的模型中重复添加。

卷轴储备和反向放绳设置参见 [XML 参考](reference.md)。固定半径模型不等同于多层绕线、绳自接触或随层数变化的半径。20 mm 直径卷线盘 $r=0.010$ m，理想关系为 $\theta=\Delta\ell/r$；10 mm 绳长变化对应 1 rad，而不是 1 deg。

## 减少抖动

先试较小控制速率、合理阻尼和可控预紧，再考虑 `control_timeconstant`、`max_contraction_rate`、`taut_transition`、`taut_hysteresis` 和路线迟滞。`visual_smoothing_timeconstant` 只平滑显示，不能修正错误路径或不稳定受力。详见[调试](debugging.md)。
