# Result 2 模型、理论基线与统计说明

## 1. Result 2.1：50 deg Faive PIP

两根主动索均显式列出 `proximal_route_surface distal_route_surface`，因此端点之间的运行时路线由插件在两个真实闭合 mesh 外壳上求解，而不是沿 seed site 直线连接。该 Result 为隔离主动驱动路线，移除了在大角度下丢失同伦走廊的两条被动韧带；这不是完整 Faive 韧带模型。

## 2. Result 2.2：官方 native 思路与 MuJoCable

两组模型共享相同的 18 个 hinge、刚度、阻尼、质量、惯量、关节限位、碰撞几何和穿绳 site。原论文 Note S3 将二维机器人表示为串联 1-DOF revolute joints，并使用 MuJoCo tendon-muscle。论文还规定每个关节的极限等于离散步长 30 deg；所有关节同时到达极限时形成紧密对数螺旋。图中的灰色实线是这一公开理论/目标，不是从论文图片反向拟合的数据。

| 项目 | MuJoCo native baseline | MuJoCable |
|---|---|---|
| 路线 | spatial tendon 依次经过固定 site | guide/孔口路线并由插件计算长度和受力 |
| 驱动 | muscle 激活量，官方生成器默认 `force=5 N` | 目标收缩量加双索预紧闭环 |
| 单向受拉 | 由 muscle/tendon 执行链决定 | 本构显式投影为 `T >= 0` |
| 摩擦 | 不表达沿多 guide 的 Capstan 张力传播 | `guide_friction_mu` 逐段传播张力 |
| 松弛状态 | 无插件 route 状态 | 输出 length、contraction、tension、taut、route status |
| 公平控制变量 | 相同关节、质量、碰撞和穿绳点 | 相同关节、质量、碰撞和穿绳点 |

两者输入物理量不同：native 输入是 muscle activation，MuJoCable 输入是收缩量。因此本实验比较模型能力和在同一目标轨迹下的响应，不把输入数值直接解释为相同执行器功。

## 3. Result 2.3：摩擦敏感性

固定模型、关节柔性、预紧、目标角和最大收缩，只改变 `guide_friction_mu = 0, 0.15, 0.30, 0.45, 0.60`。输出包含总弯曲、18 个关节角、主动索收缩和张力。该实验是参数敏感性分析；在实物辨识前，不能把某个 mu 称为已标定真实摩擦系数。

## 4. 原论文参考

Wang, Freris, and Wei, *SpiRobs: Logarithmic spiral-shaped robots for versatile grasping across scales*, Device, 2025, DOI: 10.1016/j.device.2024.100646. 采用其 Note S3 的串联关节、tendon-muscle 与 30 deg 离散关节极限作为公开基线。
