# XML 配置参考

所有配置位于 `<extension><plugin><instance>` 的 `<config key="..." value="..."/>`。名称区分大小写；未知键会在加载模型时失败。下表由文档构建器与当前公开 C++ 注册表逐键核对，不是旧版手抄的接口。

```{include} _includes/configuration.md
```

## 控制枚举

`ctrl_mode` 支持 `target_contraction`、`contraction_velocity`、`target_spool_angle`、`spool_angular_velocity`、`joint_spool_angle`。单位、符号和物理卷轴的区别见[如何驱动](actuation.md)。

## 最小模板

```xml
<instance name="rope">
  <config key="route_mode" value="surface"/>
  <config key="route_tendon" value="seed"/>
  <config key="wrap_geoms" value="pulley_wrap"/>
  <config key="home_length" value="auto_initial"/>
  <config key="stiffness" value="2000"/>
  <config key="damping" value="5"/>
  <config key="ctrl_mode" value="target_contraction"/>
</instance>
```

相应 seed 包含两个 role-1 endpoint 和一个 role-2 hint。`wrap_geoms` 不可直接写文件名，它引用已经定义的 **geom 名称**，该 geom 再引用 mesh asset。

## 配置不是传感器

`stiffness`、`capstan_mu` 等是模型参数，不会因为拖动 Control 自动改变。Control 输入到 actuator；观测来自 plugin sensor。若需要改变模型参数，先编辑 XML 并重新加载，尤其是初始化 hint、包络几何和 `home_length=auto_initial`。

所有长度使用 m，角度遵循相关接口的 rad 约定。`visual_width` 是显示线宽，seed tendon 的 `width` 是 MuJoCo 原生绳折线的视觉设置，二者不是同一个字段。
