# XML Configuration Reference

Configure an instance under `<extension><plugin><instance>` using `<config key="..." value="..."/>`. Names are case-sensitive; unknown keys cause model loading to fail. The build checks every table entry against the public C++ registration list.

```{include} _includes/configuration.md
```

## Control Enumeration

`ctrl_mode` accepts `target_contraction`, `spool_velocity`, `target_spool_angle`, `spool_angular_velocity` and `joint_spool_angle`. See [actuation](actuation.md) for units, signs and physical-spool differences.

## Minimal Instance

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

The matching seed has two role-1 endpoints and one role-2 hint. `wrap_geoms` names **geoms**, not files. A mesh geom references its mesh asset separately.

## Configuration versus Observation

`stiffness` and `capstan_mu` are parameters, not Control sliders. Actuators receive control; plugin sensors report state. Edit XML and reload when changing initialization hints, route geometry or automatic home length.

Lengths are in meters and the relevant angle interfaces use radians. `visual_width` controls runtime line display. Native seed tendon `width` controls a different visual object; the two are not interchangeable.
