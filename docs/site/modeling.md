# MJCF 建模结构

## 三层模型不要混在一起

| 层 | 你定义什么 | 谁求解 |
|---|---|---|
| 机构 | body、joint/freejoint、质量、惯量、碰撞 geom/flex、地面 | MuJoCo |
| 绳路 | endpoint、hint、guide、seed、wrap_geoms | native 时由 MuJoCo；surface 时由 MuJoCable |
| 传动 | 自由长度、刚度、阻尼、余量、预紧、控制与摩擦 | MuJoCable 本构与力映射 |

先在无绳状态验证机构和碰撞，再增加一条绳。**插件不自动创建滚动副，也不会消除你在 MJCF 里保留的 hinge。** 若想比较真实接触滚动与虚拟铰链，必须分别建立两种机构，不能只改 actuator 名称。

## 一个实例对应一根绳

```xml
<size nuser_site="1"/>
<extension>
  <plugin plugin="mujoco.cable.unilateral">
    <instance name="flexor">
      <config key="route_mode" value="surface"/>
      <config key="route_tendon" value="flexor_seed"/>
      <config key="wrap_geoms" value="prox_wrap dist_wrap"/>
      <config key="home_length" value="auto_initial"/>
      <config key="stiffness" value="800"/>
      <config key="damping" value="4"/>
      <config key="slack" value="0.0002"/>
      <config key="pretension_offset" value="0.0008"/>
      <config key="max_tension" value="15"/>
    </instance>
  </plugin>
</extension>
```

这里的名称只是示意；运行需要完整 body/site/tendon 定义，可从[案例 ZIP](gallery.md)或[最小 XML](quickstart.md)开始，不要把带省略对象的片段当作完整模型加载。

## site 角色表

| `site user` 首槽 | 角色 | 运行时必须经过 | 计入绳长与受力 | 放在哪里 |
|---:|---|:---:|:---:|---|
| 1 | 真实端点 | 是 | 是 | 真实系绳位置，seed 首尾，避免在实体内部 |
| 2 | 初始化 hint | 否 | 否 | 希望绕行的一侧；每个 hint 对应一个 wrap geom |
| 3 | 真实 guide | 是 | 是 | 真实孔口、滑环、导管出口，不可凭空悬空 |
| 0 | 普通标记 | 否 | 否 | 视觉/测量参考，不放入 surface seed |

`site` 的 `pos` 相对于所属 body，不是默认世界坐标。移动父 body、改变 `quat/euler` 或 mesh 编译变换后，应在 viewer 中重新检查它与实体的相对位置。

```xml
<tendon>
  <spatial name="flexor_seed" width="0.000000001" rgba="0.9 0.15 0.1 1">
    <site site="prox_end"/>
    <site site="prox_hint"/>
    <site site="dist_hint"/>
    <site site="dist_end"/>
  </spatial>
</tendon>
```

hint 顺序必须与 `wrap_geoms="prox_wrap dist_wrap"` 一一对应；同一 geom 可以重复，表达多次访问。seed 不应额外配置弹簧、阻尼或长度约束，否则它会沿自己的折线路径产生额外物理作用。

## 主动索与被动韧带

主动 surface 索增加 plugin actuator，并指向同一 seed，使用 `gear="0"`。被动 surface 绳只保留实例和 sensor，不写 actuator；没有输入时 `contraction=0`，依然通过 passive 阶段受力。

没有 actuator 不等于绳长固定。若模型靠韧带形成柔顺关节，要同时检查有限绳刚度、余量、预张力、接触与剩余自由度。初版应使用较低刚度验证可动性，再标定材料参数。

## 三种 mesh 职责

| 几何 | 用途 | 常见错误 |
|---|---|---|
| 视觉 mesh | 看见机器人外观 | 误以为外观有孔就一定可穿孔碰撞 |
| 刚体碰撞 geom / rigid flex | 约束实体之间接触 | 普通非凸 mesh 被凸化后孔或凹槽消失 |
| route geom | 阻止求解绳路穿透已配置表面 | 与视觉 mesh 未对齐，或漏列障碍 |

三者可以复用一个形状，也可以分别简化，但必须对齐并记录差异。对数螺线和手指中的弹性 hinge 是柔性梁的降阶模型，不是细梁连续体有限元。

## 参数如何估计

均匀细绳的轴向小应变刚度可从 $k=EA/L_0$ 估算，单位 N/m；不是直接把材料弹性模量 E（Pa）填入 `stiffness`。泊松比不是当前中心线拉索插件的输入参数。阻尼、有效面积、编织结构和接头柔顺应由拉伸/回弹数据辨识。

避免把“长度不可变”理解为把 k 调到无穷大。很大刚度需要步长收敛、质量与能量检查；严格不可伸长约束与有限刚度绳的数值语义不同。
