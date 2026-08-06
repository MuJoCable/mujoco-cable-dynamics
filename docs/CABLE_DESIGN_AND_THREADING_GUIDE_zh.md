# MuJoCable 建模、选型与穿绳指南

## 1. 先区分三层模型

1. **MuJoCo 刚体与接触层**：body、joint/freejoint、质量、惯量、碰撞 geom、地面和
   自碰撞。插件不会替代刚体碰撞。
2. **绳路几何层**：端点、初始化 hint、永久 guide、圆柱或 mesh 包络面。
3. **绳索本构与控制层**：自由长度、刚度、阻尼、松弛、预紧、最大张力、卷轴或
   收缩控制、摩擦与传感器。

先让接触和关节在无绳状态下稳定，再布置绳路，最后增加预紧和控制。否则很难判断
运动异常来自碰撞、路线还是本构。

## 2. 路径点角色

模型需要 `<size nuser_site="1"/>`，并用第一个 `site user` 值区分：

| 角色 | XML | 运行时含义 |
|---|---|---|
| 固定端点 | `user="1"` | 绳端，参与受力 |
| 初始化 hint | `user="2"` | 只指定包络顺序和绕行侧，初始化后不受力 |
| 真实 guide/孔口 | `user="3"` | 运行时必须经过，参与受力和局部摩擦 |
| 普通 site | `user="0"` | 无插件语义 |

seed tendon 只保存顺序，建议 `width="0.000000001"`，避免 MuJoCo 把经过 hint 的
直线误认为真实绳路。真实路径由插件 scene 回调绘制。

## 3. 按机械结构选择路线

### 3.1 固定折点或简单直线：`native`

适合张拉整体、只经过固定 guide 的机构和与原生 tendon 对照。参考 Demo 17-19。

```xml
<config key="route_mode" value="native"/>
<config key="route_tendon" value="cable_seed"/>
```

若只需要 MuJoCo 已支持的空间 tendon，优先使用原生 tendon；插件价值主要在单边
松弛、自由长度、卷轴状态和统一传感器。

### 3.2 圆柱滑轮：`surface` + role-2 hint

参考 Demo 15。端点应位于圆柱外；hint 放在希望绳子绕行的上侧或下侧，不必精确位于
切点。一个 hint 对应一个 `wrap_geom`。

```xml
<site name="start" user="1" .../>
<site name="upper_hint" user="2" .../>
<site name="end" user="1" .../>
<spatial name="route_seed" width="0.000000001">
  <site site="start"/><site site="upper_hint"/><site site="end"/>
</spatial>
<config key="route_mode" value="surface"/>
<config key="route_tendon" value="route_seed"/>
<config key="wrap_geoms" value="pulley_wrap"/>
```

多滑轮按穿绳顺序增加 hint 和 geom。Demo 10 展示连续多表面路线；两个相邻表面之间
由共同切线连接，而不是把 hint 当折点。

### 3.3 闭合 mesh 包络

- `convex_surface`：闭合凸 mesh，最稳定；参考 Demo 14。
- `taut_obstacle`：闭合非凸障碍上的最短无穿透受拉路线，可跨过凹槽。
- `guided_surface`：同伦引导的表面走廊，适合滚动关节韧带；参考 Demo 25。

```xml
<config key="route_mode" value="surface"/>
<config key="mesh_route_mode" value="guided_surface"/>
<config key="wrap_geoms" value="proximal_surface distal_surface"/>
```

mesh 必须闭合、流形、法向一致且无自交。视觉 mesh、碰撞 mesh 和绳路 mesh 可以是
不同的简化版本，但必须在文档中明确各自作用。先用低面数代理调通，再替换为真实
闭合外表面。

### 3.4 穿孔与孔口

STL/OBJ 可以保留几何孔，但普通 MuJoCo rigid mesh 碰撞通常使用凸化表示。需要真实
非凸孔碰撞时可用 rigid flex（Demo 31）。当前无质量中心线绳不会自动发现孔并穿入，
应在孔入口/出口放置 role-3 guide 明确拓扑（Demo 32、Demo 28、Demo 33）。

```xml
<site name="eyelet" user="3" .../>
<config key="guide_friction_mu" value="0.10"/>
<config key="capstan_direction" value="forward"/>
```

guide 是理想点式孔口。它不表示孔径、孔壁分布接触、卡滞或磨损。研究这些现象时需
有厚度/有质量绳模型，不能只调 `guide_friction_mu`。

### 3.5 被动韧带与滚动关节

被动绳不写 `<actuator>`。用 `pretension_offset` 和有限刚度形成恢复力，绳子
伸长时受拉，缩短后松弛，不会推刚体。Demo 13/16 展示八字韧带；Demo 20 增加上下
拮抗控制索。滚动运动来自“曲面接触 + 多条单边绳”共同作用，不是插件自动创建的
joint。

### 3.6 拮抗绳、余量与差动控制

一侧缩短时，对侧路径通常增长。若对侧卷轴不放绳，会产生不希望的拮抗张力。设每侧
初始余量 `R`：

\[
L_{free}=L_{home}+R-c,
\]

可用 `pretension_offset="-R"` 实现。Demo 33 先令两侧控制都从 `0` 增至 `R`
收走余量，再使用 `c_+=R+u, c_-=R-u` 差动收放。`R` 应大于最大运动范围内对侧
路径增长，并通过传感器验证释放侧张力回到零。

### 3.7 卷轴与自由滑轮

卷扬机使用 `spool_joint`、半径、方向和预绕储备控制自由长度，必要时启用卷轴
反力矩。自由滑轮需要真实 hinge、质量和惯量；Demo 29 用速度定向摩擦验证
`tau=R(T_2-T_1)`、轮缘速度、绳速和耗散。固定 `capstan_direction` 适合已知
滑动方向；会反转的机构应使用速度定向模式。

## 4. 材料参数

当前本构是单边 Kelvin-Voigt：

\[
T=\operatorname{clip}\left(k[L-L_{free}-s]_+ + c\dot L,0,T_{max}\right).
\]

- `stiffness` 单位 N/m，可由 `EA/L` 初估；当前模型不单独输入泊松比。
- `damping` 单位 N s/m，应通过自由衰减或张力阶跃标定。
- `slack` 是小的本构死区，不应代替厘米级放绳余量。
- `pretension_offset>0` 缩短自由长度并产生预张力；负值增加余量。
- 近似不可伸长绳仍使用有限大刚度，需减小时间步或启用隐式柔顺模式；不要使用
  无限刚度。

## 5. 调试顺序

1. seed tendon 设极细，只显示 endpoint/hint/guide。
2. 暂时关闭重力和 actuator，确认 body 接触无穿透、freejoint/hinge 正确。
3. 开启 `--show-route-debug --show-cable-state`，检查路线状态和残差。
4. 保持低刚度、低预紧，移动一个 body 扫描整个姿态范围。
5. 检查每条自由段不穿透全部 `wrap_geoms`，多表面界面近似相切。
6. 逐步增加刚度、阻尼、预紧和控制速率；记录时间步收敛。
7. 用虚功 `qfrc approx -T dL/dq`、静力平衡和 Capstan 比验证力学。
8. 最后再增加自碰撞、地面和复杂控制。松弛绳变灰是预期状态；无质量绳不会自然
   下垂。

完整启动命令见 [RUN_ALL_DEMOS_zh.md](RUN_ALL_DEMOS_zh.md)。
