# 表面摩擦与孔口

## 两类系数

`capstan_mu` 用于包络表面的转角传播，`guide_friction_mu` 用于真实 role-3 guide 的折转。它们是绳路模型的系数，**不等同于 geom 的 `friction`**：后者负责 MuJoCo 刚体接触，不会自动赋给绳。

```xml
<config key="capstan_mu" value="0.15"/>
<config key="guide_friction_mu" value="0.10"/>
<config key="capstan_direction" value="forward"/>
```

同一实例使用统一的表面/guide 系数，当前不是逐个 guide 独立指定材料的接口。分段建模时仍要保证它确实代表一根连续绳的力学耦合，不可仅拆成若干互不耦合弹簧就宣称摩擦相同。

## 方向与张力传播

沿 seed 的起点到终点编号。对正转角 $\phi_i\ge0$：

$$T_{i+1}=T_i\exp(\sigma_i\mu_i\phi_i).$$

| `capstan_direction` | 当前实现的含义 |
|---|---|
| `forward` | 沿点序指数衰减，$\sigma=-1$ |
| `reverse` | 沿点序指数增长，$\sigma=+1$ |
| `velocity` | 根据估计材料滑速与表面速度，用 tanh 连续决定符号 |

不要只看名字猜“哪一端是高张力端”。先画 seed 点序，再核对电机收绳方向和传感器所在的起点侧。T 的默认报告不是“所有绳段相同的张力”。

速度模式使用 `capstan_velocity_scale`（m/s）正则化；正收缩表示材料进入 seed 第一端。当前推进阶段估计收缩速率，因此控制跳变时不能把该估计当作理想瞬时测速。

## 不是严格静摩擦求解器

Euler–Eytelwein 关系给出充分滑移/极限状态下的比值。速度正则化在近零滑速回到接近等张力，不能据此宣称实现完整静摩擦区间、粘滞记忆或精确 stick-slip 互补条件。

实验标定时记录绕角、速度、预紧、材料与加载方向，不要仅从一次末端弯曲反推唯一 μ。更大摩擦可能改变源端载荷、远端可用力和关节弯曲分配，而不只是“让运动更慢”。

## 自由滑轮为什么会被带动

给滑轮 body 一个有惯量的 hinge，并保留轴承阻尼；插件接触节点施力可产生转矩。简单平面关系为

$$I\dot\omega+b\omega=R(T_{\rm source}-T_{\rm load}).$$

无摩擦等张力时不会凭空产生这个切向驱动转矩。存在滑移时，两轮转角不必严格满足半径比；若用 equality 强制无滑移，要明确那是理想约束基线。参考[自由滑轮](./_generated/free_sheave.md)。

## 孔口应该怎样建

1. 入口和出口放在真实孔口位置，绑定到正确 body。
2. 以 role-3 guide 记录必须穿过的路径。
3. 用 `guide_friction_mu` 与局部折角近似绳—孔口的传力损失。
4. 单独验证指骨之间或孔壁的刚体碰撞模型。

绳直线通过理想点且折角为零时，该点不产生转角型 Capstan 损失。单个 guide 不表示孔径、孔道长度、孔壁分布压力、卡滞或磨损。若要研究这些现象，需要更细的接触/有直径绳模型。

仓库中的孔口示例可从 [Demo 31/32 的说明](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/DEMO_31_32_EYELET_FRICTION.md) 开始；单指与 SpiRobs 案例展示其在机器人里的用法。
