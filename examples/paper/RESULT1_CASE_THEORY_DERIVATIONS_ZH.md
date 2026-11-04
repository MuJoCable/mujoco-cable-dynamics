# Result 1 七个滑轮工况的理论推导与仿真差异

## 1. 文档目的

本文档是论文 Result 1 的计算参考文件。正文表格只保留工况名称及理论—仿真差异；本文档逐一说明：

1. 验证的物理关系；
2. 理论模型及成立条件；
3. 参数和代入计算；
4. 理论值、仿真值和误差；
5. 误差指标的精确定义。

所有仿真数值来自 `experiment0814` 中已经冻结的逐时刻 CSV 和 summary JSON。除特别说明外，重力加速度取

$$
g=9.81\ \mathrm{m/s^2}.
$$

## 2. 统一符号与误差定义

- $c(t)$：驱动端收绳量；
- $h(t)$：负载或动滑轮的竖直位移；
- $T_s,T_l$：绳索源端和负载端张力；
- $R$：滑轮半径；
- $\beta$：绳索包角；
- $\mu$：绳索—滑轮摩擦系数；
- $\omega$：滑轮角速度；
- $y_k^{\mathrm{th}},y_k^{\mathrm{sim}}$：第 $k$ 个采样点的理论值和仿真值。

时间序列均方根误差定义为

$$
\operatorname{RMSE}(y)=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}
\left(y_k^{\mathrm{sim}}-y_k^{\mathrm{th}}\right)^2}.
$$

终值差和相对终值差定义为

$$
\Delta y_f=y_f^{\mathrm{sim}}-y_f^{\mathrm{th}},
\qquad
\delta_f=\frac{\Delta y_f}{\left|y_f^{\mathrm{th}}\right|}\times100\%.
$$

对于分别在不同时刻达到峰值的量，峰值差定义为

$$
\Delta y_{\max}=\max_t y^{\mathrm{sim}}(t)-\max_t y^{\mathrm{th}}(t).
$$

因此，终值差、峰值差和 RMSE 是三个不同指标，本文不将它们混为同一种“误差”。

---

## 3. 工况 1：固定滑轮，$\mu=0$

### 3.1 验证关系

该工况验证单个固定滑轮的 1:1 长度传递和无摩擦等张力关系。滑轮位置固定，负载由一段活动绳段支承，因此准静态长度兼容关系为

$$
h(t)=c(t).
$$

无摩擦时，两侧张力相等。负载质量为 $m=0.2\ \mathrm{kg}$，所以负载端理论张力为

$$
T_l=T_s=mg
=0.2\times9.81
=1.962\ \mathrm{N}.
$$

### 3.2 理论值和仿真值

最终收绳量为 $c_f=25\ \mathrm{mm}$，因此理论负载位移为

$$
h_f^{\mathrm{th}}=25.000\ \mathrm{mm}.
$$

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 最终负载位移 | 25.000 mm | 24.946 mm | $-0.054$ mm（$-0.216\%$） |
| 位移时间序列 | $h=c$ | 全轨迹 | RMSE = 0.160563 mm |
| 稳态负载端张力 | 1.962 N | 1.962 N | 数值差约 $-3.1\times10^{-14}$ N |
| 源端张力时间序列 | 1.962 N | 全轨迹 | RMSE = 0.007620 N |

以 25 mm 最大理论行程为尺度，位移归一化 RMSE 为

$$
\frac{0.160563}{25}\times100\%=0.6423\%.
$$

该工况对应视频：`result_1_1_mu_0_annotated.mp4`。

---

## 4. 工况 2：固定滑轮，$\mu=0.15$

### 4.1 验证关系

该工况验证速度方向正则化的 Capstan 张力传递。采用

$$
\frac{T_s}{T_l}
=\exp\left[
\mu\beta\tanh\left(\frac{|v_{\mathrm{rel}}|}{v_s}\right)
\right],
$$

其中 $v_s=0.01\ \mathrm{m/s}$。该固定滑轮工况的相对材料速度按照实现中的端点速度估计为

$$
|v_{\mathrm{rel}}|=1.5|\dot c|.
$$

包角在整个运动中几乎不变，其均值为

$$
\beta=2.094317\ \mathrm{rad}
=119.9955^\circ.
$$

峰值收绳速度为 $\dot c_{\max}=0.0208333\ \mathrm{m/s}$，所以

$$
|v_{\mathrm{rel}}|_{\max}
=1.5\times0.0208333
=0.03125\ \mathrm{m/s},
$$

$$
\tanh\left(\frac{0.03125}{0.01}\right)
=\tanh(3.125)
=0.9961465.
$$

理论峰值张力比为

$$
\left(\frac{T_s}{T_l}\right)_{\max}^{\mathrm{th}}
=\exp(0.15\times2.094317\times0.9961465)
=1.3674354.
$$

负载端准静态张力仍为 $T_l=mg=1.962\ \mathrm{N}$，所以源端理论峰值张力为

$$
T_{s,\max}^{\mathrm{th}}
=1.962\times1.3674354
=2.682908\ \mathrm{N}.
$$

### 4.2 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 峰值张力比 | 1.367435 | 1.361352 | $-0.006084$（$-0.4449\%$） |
| 峰值源端张力 | 2.682908 N | 2.672718 N | $-0.010190$ N（$-0.3798\%$） |
| 源端张力时间序列 | 正则化 Capstan 预测 | 全轨迹 | RMSE = 0.016479 N |
| 位移时间序列 | $h=c$ | 全轨迹 | RMSE = 0.322085 mm |

收绳停止后 $v_{\mathrm{rel}}=0$，因此 $\tanh(0)=0$，理论张力比连续回到 1；仿真终值张力比同样回到 1。

该工况对应视频：`result_1_1_mu_0p15_annotated.mp4`。

---

## 5. 工况 3：自由滑轮，$\mu=0$

### 5.1 验证关系

该工况解除滑轮转动自由度，验证无张力差时滑轮不会获得切向驱动力矩。滑轮转动方程为

$$
I_{\mathrm{eff}}\dot\omega+b\omega
=R(T_s-T_l),
$$

其中

$$
I_{\mathrm{eff}}=3.0\times10^{-4}\ \mathrm{kg\,m^2},
\quad
b=0.005\ \mathrm{N\,m\,s/rad},
\quad
R=0.05\ \mathrm{m}.
$$

当 $\mu=0$ 时，Capstan 关系给出 $T_s=T_l$，因此

$$
R(T_s-T_l)=0.
$$

在初始条件 $\omega(0)=0$ 下，微分方程的解为

$$
\omega(t)=0.
$$

### 5.2 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 峰值滑轮角速度 | 0 rad/s | $3.7449\times10^{-15}$ rad/s | $3.7449\times10^{-15}$ rad/s |
| 绳索驱动力矩 | 0 N·m | 数值残差 | RMSE = $4.5149\times10^{-17}$ N·m |
| 峰值张力比 | 1 | $1+2.2\times10^{-16}$ | 浮点舍入量级 |

该工况对应视频：`result_1_1_b_mu_0_annotated.mp4`。

---

## 6. 工况 4：自由滑轮，$\mu=0.15$

### 6.1 验证关系

该工况验证张力差、滑轮惯量和表面速度之间的耦合。相对滑移速度为

$$
v_{\mathrm{rel}}=-1.5\dot c+R\omega,
$$

滑轮转速升高后，$R\omega$ 抵消部分绳索材料速度，使有效张力比随运动状态变化。理论参考由以下耦合方程逐时刻积分得到：

$$
\frac{T_s}{T_l}
=\exp\left[-\mu\beta
\tanh\left(\frac{v_{\mathrm{rel}}}{v_s}\right)\right],
$$

$$
I_{\mathrm{eff}}\dot\omega+b\omega
=R(T_s-T_l),
$$

$$
T_l=mg=1.962\ \mathrm{N}.
$$

参数为

$$
I_{\mathrm{eff}}=3.0\times10^{-4}\ \mathrm{kg\,m^2},
\quad b=0.005\ \mathrm{N\,m\,s/rad},
\quad R=0.05\ \mathrm{m},
\quad \mu=0.15.
$$

由于 $T_s/T_l$ 本身依赖 $\omega$，这里的理论基线是上述已定义动力学方程的数值积分，而不是常张力差条件下的单指数闭式解。

### 6.2 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 峰值滑轮角速度 | 0.605586 rad/s | 0.605326 rad/s | $-2.5975\times10^{-4}$ rad/s（$-0.0429\%$） |
| 峰值绳索力矩 | 0.003040943 N·m | 0.003040198 N·m | $-7.4468\times10^{-7}$ N·m（$-0.0245\%$） |
| 力矩时间序列 | 动力学积分 | 全轨迹 | RMSE = $4.0876\times10^{-5}$ N·m |
| 峰值张力比 | 1.0309984 | 1.0309804 | $-1.8039\times10^{-5}$（$-0.00175\%$） |

该工况对应视频：`result_1_1_b_mu_0p15_annotated.mp4`。

---

## 7. 工况 5：25 mm 卷筒—50 mm 滑轮

### 7.1 验证关系

卷筒和滑轮半径分别为

$$
r_w=0.025\ \mathrm{m},
\qquad
r_p=0.050\ \mathrm{m}.
$$

理想无滑移圆周行程相等：

$$
s=r_w\theta_w=-r_p\theta_p.
$$

因此理论角度比为

$$
\frac{\theta_p}{\theta_w}
=-\frac{r_w}{r_p}
=-0.5.
$$

仿真终值卷筒角为 $\theta_w=1.597547107\ \mathrm{rad}$，代入理论关系得到

$$
\theta_p^{\mathrm{th}}
=-0.5\times1.597547107
=-0.798773554\ \mathrm{rad},
$$

$$
s_f^{\mathrm{th}}
=0.025\times1.597547107
=0.039938678\ \mathrm{m}
=39.938678\ \mathrm{mm}.
$$

### 7.2 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 最终滑轮角 | $-0.798773554$ rad | $-0.798773553$ rad | $+1.0722\times10^{-9}$ rad |
| 角度时间序列 | $\theta_p=-0.5\theta_w$ | 全轨迹 | RMSE = $9.4693\times10^{-6}$ rad |
| 最终圆周行程 | 39.938678 mm | 39.938678 mm | $-5.36\times10^{-8}$ mm |
| 圆周行程时间序列 | $r_w\theta_w=-r_p\theta_p$ | 全轨迹 | RMSE = $4.7347\times10^{-4}$ mm |
| 最终负载位移 | 39.938678 mm | 39.832587 mm | $-0.106091$ mm（$-0.2656\%$） |
| 负载位移时间序列 | $h\approx s$ | 全轨迹 | RMSE = 0.091502 mm |

该工况对应视频：`result_1_1_c_winch_annotated.mp4`。

---

## 8. 工况 6：单动滑轮，$n=2$

### 8.1 验证关系

动滑轮由两段绳索支承。当动滑轮上升 $h$ 时，两段活动绳段各缩短 $h$，因此长度兼容关系为

$$
c=2h,
\qquad
h=\frac{c}{2}.
$$

动滑轮质量和负载质量分别为

$$
m_p=0.08\ \mathrm{kg},
\qquad
m_l=0.20\ \mathrm{kg}.
$$

理想准静态力平衡为

$$
2T=(m_p+m_l)g,
$$

所以

$$
T^{\mathrm{th}}
=\frac{(0.08+0.20)\times9.81}{2}
=1.3734\ \mathrm{N}.
$$

最终收绳量为 $c_f=20\ \mathrm{mm}$，因此

$$
h_f^{\mathrm{th}}=10.000\ \mathrm{mm}.
$$

### 8.2 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 最终动滑轮位移 | 10.000 mm | 10.174848 mm | $+0.174848$ mm（$+1.7485\%$） |
| 动滑轮位移时间序列 | $h=c/2$ | 全轨迹 | RMSE = 0.142472 mm |
| 最终负载位移 | 10.000 mm | 10.184683 mm | $+0.184683$ mm（$+1.8468\%$） |
| 负载位移时间序列 | $h=c/2$ | 全轨迹 | RMSE = 0.435368 mm |
| 最终源端张力 | 1.3734 N | 1.400837 N | $+0.027437$ N（$+1.9977\%$） |
| 张力时间序列 | 1.3734 N | 全轨迹 | RMSE = 0.129244 N |
| 动滑轮—负载连接索长度 | 常数 | 最大漂移 4.170 µm | 4.170 µm |

该工况对应视频：`result_1_2_moving_pulley_annotated.mp4`。

---

## 9. 工况 7：带惯量和轴承阻尼的 Atwood 装置

### 9.1 等效动力学

左右质量、滑轮半径、有效转动惯量和轴承阻尼为

$$
m_L=0.1168032\ \mathrm{kg},
\qquad
m_R=0.2341344\ \mathrm{kg},
$$

$$
R=0.05\ \mathrm{m},
\qquad
I_{\mathrm{eff}}=3.0\times10^{-4}\ \mathrm{kg\,m^2},
\qquad
b=0.005\ \mathrm{N\,m\,s/rad}.
$$

在接触前，令右侧质量向下速度为 $v$，并采用理想无滑移关系 $\omega=v/R$。滑轮惯量折算到平动方向后，等效质量为

$$
M_{\mathrm{eff}}
=m_L+m_R+\frac{I_{\mathrm{eff}}}{R^2}
=0.1168032+0.2341344+
\frac{0.0003}{0.05^2}
=0.4709376\ \mathrm{kg}.
$$

轴承阻尼折算到平动方向为

$$
\gamma=\frac{b}{R^2}
=\frac{0.005}{0.05^2}
=2.0\ \mathrm{kg/s}.
$$

重力驱动力为

$$
F=(m_R-m_L)g
=(0.2341344-0.1168032)\times9.81
=1.151019072\ \mathrm{N}.
$$

因此接触前动力学方程为

$$
M_{\mathrm{eff}}\dot v+\gamma v=F.
$$

在 $v(0)=0$ 下，定义

$$
\tau=\frac{M_{\mathrm{eff}}}{\gamma}
=0.2354688\ \mathrm{s},
\qquad
v_\infty=\frac{F}{\gamma}
=0.575509536\ \mathrm{m/s},
$$

则解析速度和位移为

$$
v(t)=v_\infty\left(1-e^{-t/\tau}\right),
$$

$$
x(t)=v_\infty
\left[t-\tau\left(1-e^{-t/\tau}\right)\right].
$$

初始加速度为

$$
a(0)=\frac{F}{M_{\mathrm{eff}}}
=2.444101\ \mathrm{m/s^2}.
$$

由

$$
T_L=m_L(g+\dot v),
\qquad
T_R=m_R(g-\dot v),
$$

得到初始理论张力

$$
T_L(0)=1.431318\ \mathrm{N},
\qquad
T_R(0)=1.724610\ \mathrm{N}.
$$

### 9.2 理论落地时间

右侧质量底面到地面的初始距离为

$$
x_{\mathrm{floor}}=0.111\ \mathrm{m}.
$$

求解

$$
0.575509536
\left[t-0.2354688
\left(1-e^{-t/0.2354688}\right)\right]
=0.111
$$

得到理论首次落地时间

$$
t_{\mathrm{land}}^{\mathrm{th}}
=0.381812138\ \mathrm{s}.
$$

### 9.3 理论值和仿真值

| 量 | 理论值 | 仿真值 | 差异 |
|---|---:|---:|---:|
| 首次落地时间 | 0.381812 s | 0.346000 s | $-0.035812$ s（$-9.3795\%$） |
| 接触前右块速度 | 解析速度曲线 | 接触前轨迹 | RMSE = 0.066776 m/s |
| 接触前滑轮角速度 | $\omega=v/R$ | 接触前轨迹 | RMSE = 1.405832 rad/s |
| 接触后峰值地面约束力 | 不属于接触前解析式 | 15.005731 N | 作为碰撞响应单独报告 |

速度 RMSE 的比较区间定义为

$$
0\le t<\min
\left(t_{\mathrm{land}}^{\mathrm{th}},
t_{\mathrm{land}}^{\mathrm{sim}}\right)
=0.346\ \mathrm{s}.
$$

在最后一个接触前采样点 $t=0.344\ \mathrm{s}$：

$$
v^{\mathrm{th}}=0.441978\ \mathrm{m/s},
\qquad
v^{\mathrm{sim}}=0.601334\ \mathrm{m/s}.
$$

该工况对应视频：`result_1_3_atwood_landing_annotated.mp4`。

---

## 10. 正文两列表采用的差异指标

| ID | Case | 正文保留的差异 |
|---|---|---|
| FP0 | Fixed, $\mu=0$ | 位移 RMSE 0.161 mm；源端张力 RMSE 0.0076 N |
| FP15 | Fixed, $\mu=0.15$ | 峰值张力比差 $-0.0061$；源端张力 RMSE 0.0165 N |
| FS0 | Free sheave, $\mu=0$ | 峰值角速度 $3.74\times10^{-15}$ rad/s；力矩 RMSE $4.51\times10^{-17}$ N·m |
| FS15 | Free sheave, $\mu=0.15$ | 峰值角速度差 $-2.60\times10^{-4}$ rad/s；力矩 RMSE $4.09\times10^{-5}$ N·m |
| WS | Winch--sheave | 圆周行程 RMSE $4.73\times10^{-4}$ mm；负载位移 RMSE 0.0915 mm |
| MP | Moving pulley | 动滑轮/负载位移 RMSE 0.142/0.435 mm |
| AT | Atwood, pre-impact | 接触时间差 $-0.0358$ s；速度 RMSE 0.0668 m/s |

## 11. 数据来源

- 固定滑轮：`result_1_1/data/result_1_1_mu_0*_timeseries.csv` 和对应 summary JSON；
- 自由滑轮：`result_1_1_b/data/result_1_1_b_mu_0*_timeseries.csv` 和对应 summary JSON；
- 卷筒—滑轮：`result_1_1_c/data/result_1_1_c_winch_timeseries.csv`；
- 动滑轮：`result_1_2/data/result_1_2_moving_pulley_timeseries.csv`；
- Atwood：`result_1_3/data/result_1_3_atwood_landing_timeseries.csv`；
- 理论计算实现：`run_result1_experiments.py` 中的 `apply_theory`。

本文档中的“仿真值”均指冻结工况输出，不是人工从图中读取的近似值。
