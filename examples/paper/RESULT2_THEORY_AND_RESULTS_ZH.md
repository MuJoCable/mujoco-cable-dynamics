# Result 2 理论、模型匹配与统计结果

## 1. 统一动力学边界

对于关节坐标 q，绳路长度为 L(q)，几何力臂和广义力分别为

```text
r(q) = -dL/dq,                  tau_c = T r(q) = -T dL/dq.
T = clip(k [L-(L0-c)-s]_+ + d Ldot, 0, Tmax).
```

MuJoCo 负责刚体惯性、关节弹簧阻尼和接触；MuJoCable 负责运行时绳路、单向张力、分段摩擦和由路径 Jacobian 产生的节点力。

## 2. Result 2.1：Faive PIP 50 deg

理想基线给两个虚拟铰链各 25 deg 指令，总目标为 50 deg。MuJoCable 使用 10 mm 收缩；flexor 与 extensor 均按 proximal mesh -> distal mesh 的顺序包络两个真实外壳。为隔离主动路线，大角度下会失去有效同伦走廊的两条被动韧带未包含在本项中。

- 理想基线最终角：49.994 deg；
- MuJoCable 最终角：52.481 deg；
- 峰值张力：12.181 N；
- mesh 接触保持率：99.92%；
- 最大 route status：0（0 为 valid）。

这证明当前主动双表面路线可以覆盖 50 deg，但不能被表述为完整 Faive 韧带模型已经在 50 deg 全部有效。

## 3. Result 2.2：原生 tendon 与 MuJoCable

原论文 Note S3 使用串联 1-DOF hinge 和 MuJoCo spatial tendon + muscle。这里保留 Demo 49 的 18 个关节、网格、质量、惯量、限位、穿绳点、刚度和阻尼，仅替换绳索执行链。官方生成器默认 muscle force 为 5 N，本基线沿用该值。

| 指标 | Native tendon + muscle | MuJoCable (mu=0.15) |
|---|---:|---:|
| 保持段总弯曲 (deg) | -521.833 | -522.970 |
| 跟踪 RMSE (deg) | 24.184 | 25.070 |
| 峰值主动索力 (N) | 0.644 | 2.538 |
| 峰值收绳/路线缩短 (mm) | 85.041 | 92.694 |

论文给出的离散关节极限为每节 30 deg，18 节的形式上限为 540 deg；实验目标使用 530 deg，为接近紧密对数螺旋且避免长期顶住全部关节限位。灰色实线表示这一公开目标/极限，不是从论文图片数字化得到的实测曲线。

## 4. Result 2.3：摩擦改变运动分配

插件在每个 guide 上使用正则化 Capstan 关系

```text
T_(i+1) / T_i = exp(mu beta_i tanh(v_rel / v_s)),
```

其中 beta_i 为局部转角。摩擦使远端可用张力降低，因而同一目标需要更大源端张力和行程，并改变 18 个关节的弯曲分配。

| mu | 保持段总弯曲 (deg) | 峰值张力 (N) | 峰值收绳 (mm) |
|---:|---:|---:|---:|
| 0 | -525.597 | 1.856 | 87.590 |
| 0.15 | -522.970 | 2.538 | 92.694 |
| 0.3 | -502.496 | 6.553 | 98.907 |
| 0.45 | -438.513 | 7.959 | 96.553 |
| 0.6 | -392.338 | 7.951 | 90.041 |

`mu=0.60` 是敏感性压力工况，不代表已标定材料。实物辨识前只能得出参数敏感性结论。

弯曲状态叠加图见 `result_2_3/figures/result_2_3_friction_bending_states`。五条曲线使用保持阶段的逐关节平均角重建平面中心线，横纵坐标采用相同物理尺度。对应关节点坐标保存在 `result_2_3/data/result_2_3_hold_shape_coordinates.csv`；五个工况的 MuJoCo 最终状态截图位于 `result_2_3/screenshots/`。

## 5. Result 2.2c：单指运动与摩擦辅助实验

为了确认摩擦效应并非只出现在 18 节对数螺旋机器人中，Result 2.2 还归档
Demo 42 的三关节六索手指。该模型由一对主动差动索和四条被动耦合索驱动，
视频将完整斜视图与弯曲平面视图分开录制，每个视角单独占用 `960 x 540` 画幅；
伸直、两个方向的运动极值和回程姿态均应在画面内。

已有 guide friction 扫描在 `mu_p=0.05` 下改变 `mu_g`：`mu_g=0` 时 3 s
总弯曲为 `269.271 deg`，`mu_g=0.30` 时降为 `214.349 deg`。已有 pulley
friction 扫描在 `mu_g=0.20` 下改变 `mu_p`：`mu_p=0` 时总弯曲为
`251.615 deg`，`mu_p=0.20` 时降为 `146.218 deg`，同时 Route 1 峰值张力
达到 `25 N` 上限。结果说明摩擦会提高源端负载并降低固定时间内的远端运动，
但不构成未经实物辨识的摩擦精度声明。

该辅助实验的完整目录为：

```text
result_2_2/demo42_single_finger/
  models/    result_2_2c_demo42_single_finger.xml
  videos/    斜视/弯曲平面各自独立的 raw、annotated 视频与海报帧
  data/      基线和两类摩擦扫描 CSV/JSON
  figures/   张力/关节载荷图与摩擦敏感性图
```

## 6. 关节柔性匹配

完整 18 关节参数见 `result_2_2/data/result_2_2_joint_parameters.csv`。Native 与 MuJoCable 模型在加载后逐数组比较 joint range、stiffness、damping、body mass/inertia、geom 和 site；任一不一致都会使实验脚本终止。代表值如下：

| Joint | Stiffness (N m/rad) | Damping (N m s/rad) |
|---:|---:|---:|
| 1 | 0.026053123 | 0.026053123 |
| 6 | 0.009066432 | 0.009066432 |
| 12 | 0.0023441717 | 0.0023441717 |
| 18 | 0.00083874025 | 0.00083874025 |

## 7. 结论边界

Result 2.1 证明表面包络主动索能够驱动大角度滚动接触运动；Result 2.2 比较同一柔性结构下两种绳索表示，并用 Demo 42 单指作为跨机构摩擦敏感性补充；Result 2.3 证明摩擦参数会改变行程、张力和关节角分配。它们是 simulation-to-simulation 证据，不构成 MuJoCable 更接近实物的充分证明；准确性仍需 Result 3 的同步实物数据。
