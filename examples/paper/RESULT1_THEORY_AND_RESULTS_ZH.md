# Result 1：滑轮与绳索动力学的理论—仿真验证

## 1. 验证边界

Result 1 使用已经人工确认的七个冻结 MJCF 工况，不再调用历史 Demo 15、21 或
29。验证对象是 MuJoCable 的质量为零、只受拉、可伸长绳索，以及圆柱表面包络、
速度正则化 Capstan 摩擦、自由滑轮惯量、卷轴和落地碰撞。它不是有质量绳、绳弯曲
刚度或严格静摩擦粘着模型。

| 工况 | 主要变量 | 理论基线 | 输出目录 |
|---|---|---|---|
| `result_1_1_mu_0` | 固定滑轮，`mu=0` | 等张力、1:1 位移 | `result_1_1/` |
| `result_1_1_mu_0p15` | 固定滑轮，`mu=0.15` | Euler–Eytelwein | `result_1_1/` |
| `result_1_1_b_mu_0` | 自由滑轮，`mu=0` | 零切向驱动转矩 | `result_1_1_b/` |
| `result_1_1_b_mu_0p15` | 自由滑轮，`mu=0.15` | Capstan 张力差与转动方程 | `result_1_1_b/` |
| `result_1_1_c_winch` | 25 mm 卷轴和 50 mm 滑轮 | 圆周位移守恒 | `result_1_1_c/` |
| `result_1_2_moving_pulley` | 单动滑轮，承载段数 `n=2` | `c=2h`、`2T=G` | `result_1_2/` |
| `result_1_3_atwood_landing` | 不等质量、惯性滑轮、落地 | 带阻尼 Atwood 方程 | `result_1_3/` |

每个工况均保存逐时刻 CSV、参数和统计 JSON、PDF/SVG/600 dpi PNG 曲线，以及
960 x 540、20 fps 的 raw/annotated 视频。曲线中理论为灰色实线，仿真为彩色虚线
并带标记；仿真层的 `zorder` 高于理论层。

## 2. 公共绳索模型和可测量量

实时包络路线给出长度 `L(q)` 和速度 `L_dot`。自由长度、有效伸长和单边张力为

$$
L_f=L_0-c-c_0,\qquad
e=L-L_f-s,\qquad
T=\max(0,k e+d\dot L),
$$

其中 `c` 为控制收缩量，`c_0` 为预张紧偏置，`s` 为松弛阈值。该 `max`
保证绳索不会产生压力。包络节点将相邻绳段的矢量合力通过 `mj_applyFT` 写入
`qfrc_passive`；插件传感器报告 12 个量：`L`、`L_dot`、`L_f`、`c`、`e`、
`T`、taut、saturated、route status、切向残差、表面残差和迭代次数。

速度方向 Capstan 模型为

$$
\frac{T_{i+1}}{T_i}
=\exp\left[\mu\beta_i\tanh\left(\frac{v_{\mathrm{rel},i}}{v_s}\right)\right].
$$

`v_s=0.01 m/s` 是固定/自由单滑轮工况的正则化尺度。它使零相对滑速时
张力比连续回到 1，而 `exp(mu beta)` 只是充分滑移时的上界。CSV 中
`tension_N` 是 seed 起点侧张力；自由负载平动自由度上的 `qfrc_passive` 模长
用于重建负载侧端点张力。

## 3. Result 1.1：固定滑轮

载荷 `m=0.2 kg`，所以无摩擦准静态理论为

$$T=mg=0.2\times9.81=1.962\ \mathrm{N},
\qquad h=c.$$

解析包角均值为 `119.996 deg`，即约 `2.094 rad`。
当 `mu=0.15` 时，充分滑移上界为

$$\exp(\mu\beta)=\exp(0.15\times2.094)\approx1.369.$$

`mu=0` 时，25 mm 收绳的位移 RMSE 为
`0.161 mm`，源端张力 RMSE 为
`0.0076 N`，稳态负载端张力为
`1.962000 N`。`mu=0.15` 时，速度
正则化公式与仿真源端张力的 RMSE 为
`0.0165 N`；理论/仿真峰值张力比分别为
`1.3674` 和
`1.3614`。这说明摩擦改变收绳过程中的
瞬态源端张力，但停止收绳且滑轮固定后，正则化模型回到等张力稳态。

[无摩擦曲线](result_1_1/figures/result_1_1_mu_0_theory_vs_simulation.pdf)；
[mu=0.15 曲线](result_1_1/figures/result_1_1_mu_0p15_theory_vs_simulation.pdf)。

## 4. Result 1.1b：带惯量的自由滑轮

自由滑轮满足

$$I_{eff}\dot\omega+b\omega=R(T_{source}-T_{load}),$$

其中 `R=0.05 m`、`b=0.005 N m s/rad`。`mu=0` 时两侧张力相等，峰值角速度仅
`3.745e-15 rad/s`，说明轴不会无输入自转。
`mu=0.15` 时，滑轮由张力差被动带动，峰值角速度为
`0.605 rad/s`；Capstan 预测与从
`qfrc_passive+b omega` 恢复的绳索转矩 RMSE 为
`4.088e-05 N m`。自由滑轮转动后会
降低绳—轮相对滑速，因此其峰值张力比显著小于固定滑轮，符合速度正则化模型。

[自由滑轮 mu=0.15 曲线](result_1_1_b/figures/result_1_1_b_mu_0p15_theory_vs_simulation.pdf)。

## 5. Result 1.1c：卷轴—滑轮半径传动

卷轴半径 `r_w=0.025 m`、滑轮半径 `r_p=0.050 m`。理想无滑移关系为

$$s=r_w\theta_w=-r_p\theta_p,
\qquad \frac{\theta_p}{\theta_w}=-\frac{r_w}{r_p}=-0.5.$$

该工况只有 `winch_motor` 一个控制，joint equality 只用于构造理想半径比基线。
角度约束 RMSE 为 `9.469e-06 rad`，两轮
圆周位移 RMSE 为 `0.000473 mm`。最终理论卷绕
`39.939 mm`，负载实际上升
`39.833 mm`，全程位移 RMSE 为
`0.092 mm`。

[卷轴曲线](result_1_1_c/figures/result_1_1_c_winch_theory_vs_simulation.pdf)。

## 6. Result 1.2：单动滑轮

动滑轮和负载总质量为
`m_p+m_l=0.08+0.20=0.28 kg`。
理想准静态关系为

$$c=2h,\qquad
T=\frac{(m_p+m_l)g}2
=\frac{0.28\times9.81}2=1.3734\ \mathrm{N}.$$

20 mm 收绳对应理论上升 10 mm；仿真最终动滑轮和负载分别上升
`10.175 mm` 和
`10.185 mm`，位移 RMSE 分别为
`0.142 mm`、
`0.435 mm`。黄色定长连接索最大漂移
`4.170 um`。

斜坡末端的两滑轮角度比为
`4.164`，不是理想无滑移的
2。这个差异不能用于否定 `c=2h`：当前 `target_contraction` 缩短整根自由长度，
并未定义材料从某个自由端进入的速度，也没有施加 `v_cable=R omega` 的静摩擦
粘着约束。该工况可验证承载绳段、位移和张力趋势，但滑轮角度比只是滑移诊断量。

[动滑轮曲线](result_1_2/figures/result_1_2_moving_pulley_theory_vs_simulation.pdf)。

## 7. Result 1.3：带阻尼惯性滑轮和落地事件

模型实际质量为 `m_L=0.1168032 kg`、
`m_R=0.2341344 kg`。含 armature 的有效转动惯量为
`I_eff=0.0003000 kg m^2`，因此

$$M_{eff}=m_L+m_R+\frac{I_{eff}}{R^2}
=0.4709376\ \mathrm{kg},$$

$$M_{eff}\dot v+\frac b{R^2}v=(m_R-m_L)g.$$

初始理想加速度为 `2.4441 m/s^2`，相应两侧张力为
`T_L=1.4313 N`、`T_R=1.7246 N`。右块底面距地面
`111.0 mm`；带轴承阻尼的理论落地时间为
`0.3818 s`，仿真首次接触为
`0.3460 s`，相对差
`-9.38%`。落地前右块速度 RMSE 为
`0.0668 m/s`，滑轮角速度 RMSE 为
`1.4058 rad/s`。

理论曲线在落地时终止；之后是接触冲击、绳索弹性振荡和单边松弛的分段动力学，
不能继续用无碰撞 Atwood 闭式解解释。仿真峰值地面约束力为
`15.006 N`，全程 taut 比例为
`0.530`，这正是落地后绳索会周期性松弛的数值证据。

[Atwood 落地曲线](result_1_3/figures/result_1_3_atwood_landing_theory_vs_simulation.pdf)。

## 8. 数值质量和结论边界

- 七个工况的最大 `route_status` 均为 0，未出现 degraded/invalid 路线；所有工况
  饱和比例均为 0。
- 单圆柱切向残差峰值约 `7.2e-8`，表面残差不超过浮点舍入量级。
- 当前 CSV 是确定性单次轨迹；RMSE 是时序理论—仿真误差，不是重复试验置信区间。
- `p95_step_time_ms` 包含 Python 调用和当前机器负载，仅作回归诊断，不作为跨平台
  C++ 插件性能结论。
- 最强证据是固定滑轮、自由滑轮转矩平衡、卷轴半径比和动滑轮位移关系。Atwood
  落地实验用于展示摩擦、滑移、冲击和松弛带来的理想模型偏差，论文中应单独报告
  落地前和落地后两个阶段。

## 9. 复现

```bash
cd "[your path]/mujoco-cable-dynamics/paper/icra2027_outline/experiment0814"
conda activate rope_plugin

python run_result1_experiments.py \\
  --plugin "[your path]/mujoco-cable-dynamics/build/plugin/libcable_unilateral.dylib"
```

只重算 CSV/JSON/曲线时添加 `--skip-videos`。结构和短时动力学回归使用：

```bash
python validate_result1_models.py
```
