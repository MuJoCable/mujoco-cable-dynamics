# 100_fingers 人体参数化机械手 MuJoCo 模型

## 模型范围

Demo 26 将 100_fingers 的 human preset 重建为 MuJoCo 多刚体机械手，并使用
MuJoCable 驱动。模型保留原项目的真实掌骨/指骨网格、人体尺寸、每指 4 自由度
结构和五索功能分工，但没有直接模拟一体打印的柔性韧带和外部混索轮机构。

每根手指包含掌指关节/腕掌关节的内收外展与屈伸两个自由度，以及 PIP、DIP
两个屈伸自由度，共 4 DOF；全手共 20 DOF。

每指五条可独立控制的单向受拉绳索为：

| 绳索 | 颜色 | 作用 |
|---|---|---|
| 伸肌索 | 蓝色 | 从背侧连续绕过 MCP、PIP 和 DIP |
| 外展索 | 青色 | 绕过 MCP/CMC 侧向导向面的一侧 |
| 内收索 | 品红色 | 绕过 MCP/CMC 侧向导向面的另一侧 |
| 中节屈肌索 | 橙色 | 从掌侧绕过 MCP、PIP，并固定在中节指骨 |
| 远节屈肌索 | 红色 | 从掌侧绕过 MCP、PIP、DIP，并固定在远节指骨 |

`user="2"` site 只选择圆柱包络的初始化侧，运行时不受力；`user="3"`
site 表示原机械手上真实存在的穿绳孔或导向结构，运行时保留并向所属刚体施力。

## 打开方式

协同闭合、保持和张开：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --duration 120
```

手动控制 25 条绳索：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --manual --duration 120
```

### 仅食指绳索的人工调试

Demo 27 保留完整手部网格和全部 20 个关节，但删除拇指、中指、无名指和小指的
插件实例、seed tendon、actuator、插件传感器及路线调试点。标准 Control 面板只
显示食指的五条绳：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml \
  --camera index_route_debug \
  --manual --show-route-debug --duration 120
```

`--show-route-debug` 使用红色显示真实固定端点（role 1）、黄色显示初始化
hint（role 2）、青色显示运行时真实 guide（role 3），并以半透明绿色显示解析
包络圆柱。脚本还会在终端打印每个点所属刚体及其局部 `pos`，可以直接用于修改
XML。蓝色、青色、品红色、橙色和红色细线才是插件实时计算的五条路线。

自动测试食指屈曲：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/27_cpp_plugin_100_fingers_index_cable_debug.xml \
  --camera index_route_debug --duration 120
```

### Demo 28：真实 mesh 穿绳

Demo 28 不使用绿色解析圆柱。食指近节、中节和远节 STL 先经过拓扑修复，生成闭合、
流形且无自交的 route mesh；同一个 route mesh 同时用于显示和
`wrap_geoms`。因此画面中的食指表面就是插件求解的障碍面。

源 STL 含少量非流形边和相交面，不能原样交给 `guided_surface`。修复采用
`0.5 mm` 体素化、实体填充、向内偏置的 marching-cubes 外壳和保闭合降面。
它保留整体外形和可解析的 pulley/groove，但亚体素细节不能视为精确复现。

逐条调试远节屈肌索：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/28_cpp_plugin_100_fingers_index_mesh_threading.xml \
  --camera index_route_debug \
  --manual --freeze \
  --debug-cable index_flexor_distal \
  --route-json index_flexor_distal_sites.json \
  --duration 120
```

`--debug-cable` 将其他 seed route 淡化，只显示目标路线的 endpoint/hint；
`--freeze` 固定中立位；`--route-json` 导出节点所属 body 和局部坐标。修改 XML
后重新启动 viewer 即可检查。可用名称还包括 `index_extensor`、
`index_abductor`、`index_adductor`、`index_flexor_intermediate` 以及
`index_ligament_pip_left` 等。

自动屈曲验证：

```bash
mjpython scripts/view_100_fingers_human_hand.py \
  --plugin build/plugin/libcable_unilateral.dylib \
  --model cable_plugin_demos/28_cpp_plugin_100_fingers_index_mesh_threading.xml \
  --camera index_route_debug --duration 120
```

Demo 28 还加入 MCP、PIP、DIP 左右各一条被动韧带索，共六条。它们没有
actuator，使用 `home_length="auto_initial"`、`stiffness="180"` N/m 和
`pretension_offset="0.00015"` m。当前 hinge 仍保留，所以这些绳索提供附加柔顺
恢复力，而不是单独定义关节自由度。完全用接触和韧带替代 hinge 需要另建自由刚体
关节模型并验证接触稳定性。

## 能否穿过 mesh 预留孔

可以，但必须区分“真实孔道”和“把 site 放进实体”：

1. 孔必须在 route mesh 中具有完整入口、出口和内壁。带通孔的实体仍可以是闭合
   二流形；孔腔属于实体外部的自由空间。
2. 插件不会自动发现应该穿哪个孔。必须用入口/出口附近的 role-2 hint 固定同伦
   分支；同一 mesh 可在 `wrap_geoms` 中重复，表示进入和离开同一孔道。
3. 更稳健的工程做法是在孔口设置 role-3 guide。guide 是有受力的理想导眼，
   可以强制路线穿孔，但它不是三角面接触，也不会自动计算孔内壁的分布接触。
4. 若需要孔内壁接触，route mesh 必须保留孔壁，并使用 `guided_surface` 在孔壁上
   建立 corridor；当前尚缺针对高亏格穿孔模型的系统验收测试。
5. 仅把 guide/hint 放在没有真实孔的封闭实体内部不会“打洞”，只会得到穿透并使
   `route_status=2`，该步绳力会被禁用。

推荐先用“孔口两个 role-3 guide + 实际孔道自由段”验证穿线拓扑，再升级到孔壁
表面接触。MuJoCo `site` 本身没有碰撞几何，它只是插件的拓扑提示或理想导眼。

控制值单位是米，表示目标收缩量。模型中的等效刚度为 350–700 N/m，位于源项目
给出的弹簧系统范围 0.06–1 N/mm 内。

## 调整方法

建议一次只调整一条路线，并按以下顺序修改：

1. Demo 27 先调 `index_*_wrap` 的 `pos`、`size` 和姿态，使绿色圆柱和视觉 STL 的关节
   圆弧中心及半径一致。当前圆柱是根据公开尺寸推断的，不是 STL 表面本身。
   Demo 28 无圆柱，应直接移动 role-2 hint 选择真实 mesh 的目标 groove/侧面。
2. 修改 `*_start`、`*_end` 调整两个真实固定端，并确保固定端位于刚体外部。
3. 沿目标包络侧移动 `*_hint`。它只决定初始化同伦分支，不是运行时折点。
4. 只在实体上确有穿绳孔或导向器时保留 `*_guide`。role-3 guide 是永久受力点，
   会产生可见折角；不需要的 guide 应从 seed tendon 和 XML 中一起删除。
5. role-2 hint 数量和顺序必须与插件实例的 `wrap_geoms` 一一对应。
6. 几何正确后再调整 `stiffness`、`damping`、`slack` 和
   `pretension_offset`。

食指五条 seed tendon 的 XML 顺序就是运行时拓扑顺序。以远节屈肌索为例：

```xml
<site site="index_flexor_distal_start"/>
<site site="index_mcp_flexor_distal_hint"/>
<site site="index_proximal_flexor_guide"/>
<site site="index_pip_flexor_distal_hint"/>
<site site="index_intermediate_flexor_guide"/>
<site site="index_dip_flexor_distal_hint"/>
<site site="index_flexor_distal_end"/>
```

其中三个 hint 分别对应 `index_mcp_wrap index_pip_wrap index_dip_wrap`；
两个 guide 则始终保留在最终绳路中。不要把 seed tendon 的 `width` 调粗来判断
真实路线，因为 MuJoCo 原生只会把 seed 中各 site 画成直线连接；真实包络由插件
的 `visualize` 回调绘制。

## 当前边界

- 一体打印韧带目前等效为显式关节；
- 外部非线性混索轮和串联弹簧架尚未建模；
- 绳索绕关节使用按公开关节直径建立的解析圆柱，尚未直接沿 STL 沟槽求解；
- 第一版关闭了自碰撞和抓取物接触，以单独验证布线和驱动；
- 25 条表面路线同时求解的速度明显慢于单指模型。

网格来自 100_fingers 项目并按 CC BY 4.0 再分发。OpenSCAD 源码为 GPLv3，
没有复制到本仓库。完整出处见 `THIRD_PARTY_NOTICES.md`。
