# Actuation and Cable Take-Up

## Length Is Not a Joint Command

Geometric length $L(q)$ follows the current pose and route. Free length $L_f$ is available unstretched cable, and contraction $c$ is the output of the control mapping. Positive $c$ takes up cable; negative $c$ pays it out when the control mode and `ctrlrange` permit it.

$$L_f=L_0-c-p,\qquad e=L-L_f-s.$$

Here $p$ is `pretension_offset` and $s$ is `slack`. Increasing $c$ usually raises tension; the robot then moves and changes $L$. **Controlling contraction is neither prescribing a joint angle nor directly positioning a body.**

## Control Modes

| `ctrl_mode` | Input unit | Mapping | Typical use |
|---|---|---|---|
| `target_contraction` | m | Contraction target | Direct take-up and antagonistic cables |
| `target_spool_angle` | rad | Angle times effective radius, with reserve settings | Servo/drum angle command |
| `spool_velocity` | m/s | Integrates contraction over time | Constant line-speed winch |
| `spool_angular_velocity` | rad/s | Multiplies by radius, then integrates | Commanded drum speed |
| `joint_spool_angle` | rad from a physical hinge | Reads `spool_joint` relative to its initial angle | Physical drum with inertia and bearings |

Integrated contraction in velocity modes is clamped nonnegative. These modes are not identical to signed payout in target modes. `target_spool_angle` does not create a massive drum: explicitly build a body/hinge and use `joint_spool_angle` when drum dynamics matters.

## Direct Python Control

```python
import mujoco

mujoco.mj_loadPluginLibrary("build/plugin/libcable_unilateral.dylib")
model = mujoco.MjModel.from_xml_path("minimal_surface.xml")
data = mujoco.MjData(model)
actuator = model.actuator("pull").id
for step in range(3000):
    data.ctrl[actuator] = min(0.030, data.time * 0.010)
    mujoco.mj_step(model, data)
```

Update `data.ctrl` before each step. Do not also overwrite `qpos` to help a cable-driven robot bend: that becomes kinematic playback, not the same actuation experiment.

## Antagonistic Cables and Reserve

A simple differential command is $c_+=R+u$, $c_-=R-u$. Common take-up $R$ removes initial reserve; $u$ produces differential input. The two paths need not grow equally, so these commands do not guarantee that both cables remain taut.

Initial reserve $R$ can be represented by `pretension_offset="-R"`. First take up that reserve, then apply the differential input. The constitutive dead zone `slack` is different from, for example, 90 mm of stored payout reserve.

The payout side should also track its evolving geometric path. Ignoring smoothing and damping, a starting feedback target for small pretension $T_p$ is

$$c_{\rm hold}\approx L_0-p+s-L(q)+T_p/k.$$

This is not an exact inverse model of a multi-friction transmission. Check tension, saturation, control limits and coupling on both sides. $T=0$ means no compressive cable force, not a taut cable. See the [calibrated SpiRobs cycle](_generated/calibrated.md) for its full controller and 20 mm drum CSVs.

## Share a Command

```xml
<config key="control_source_actuator" value="mcp_differential_drive"/>
<config key="control_scale" value="-1"/>
```

Another cable can read the named actuator and scale its target, for example $c_1=u$, $c_4=-u$. This reduces the number of control channels, but is **not an automatically power-conserving mechanical differential**. Explicitly model the transmission to study motor loads and distributor dynamics.

`control_scale` applies to direct target/target-angle commands; it is not a universal gear ratio for integrated velocity modes or physical spool joints.

## Physical Spools

Set `joint_spool_angle`, `spool_joint` and `spool_radius`, then drive the spool hinge with its own motor or position actuator. The cable need not receive another independent contraction command. `spool_reaction_torque` optionally applies the corresponding reaction to the shaft; do not count the same reaction twice.

See the [XML reference](reference.md) for reserve and reverse-payout settings. Constant effective radius does not model multiple winding layers, rope self-contact or radius growth. For a 20 mm diameter drum, $r=0.010$ m and $\theta=\Delta\ell/r$: 10 mm of travel is 1 rad, not 1 deg.

## Reduce Jitter

Start with moderate input rates, suitable damping and controlled pretension. Then consider `control_timeconstant`, `max_contraction_rate`, `taut_transition`, `taut_hysteresis` and route hysteresis. `visual_smoothing_timeconstant` changes display only; it cannot repair an incorrect route or unstable force. See [troubleshooting](debugging.md).
