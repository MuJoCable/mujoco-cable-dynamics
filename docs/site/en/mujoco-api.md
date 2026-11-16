# MuJoCo Reference for Cable-Driven and Parallel Robots

**Source and version.** This page reorganizes the [official MuJoCo 3.4.0 documentation (Google DeepMind)](https://mujoco.readthedocs.io/en/3.4.0/overview.html) around common robot modeling tasks. MuJoCable supplies the examples. Native interfaces are scoped to **3.4.0**; select the matching version in the [official manual](https://mujoco.readthedocs.io/en/stable/) when working with another release. MuJoCable-specific settings are marked separately.

## Choose by Mechanism

| Task | Starting point | Modeling boundary |
|---|---|---|
| Serial arm or underactuated finger | `body/joint`; add an `actuator` on the driven joint, spool or tendon | Leave passive joints unactuated. Specify physical axes and inertia in the tree. |
| Four-bar, Delta, Stewart platform or parallel gripper | Build open branches with `body/joint`; close them with `equality/connect` or `equality/weld`. Use `equality/joint` for ideal scalar couplings. | Rigid loop closure and cable tension are different constraints. A prescribed gear ratio is not gear-tooth contact or friction. |
| Cable-driven parallel robot (CDPR) or tensegrity | Use a `freejoint` or the required planar joints for the platform; define each cable with `tendon/spatial` or a MuJoCable instance. | Cables pull only. Do not replace slack cables with bilateral equalities. Check tension feasibility, pretension and motion after loss of tension. |
| Simple cable or pulley baseline | Ordered `site` and sphere/cylinder `geom` entries in native `tendon/spatial`; `pulley` can define branch length scaling. | Native `pulley` is a routing branch element, not a sheave with inertia, a bearing and a groove. |
| Irregular surfaces, segment friction or payout | MuJoCable `surface` routing; use a body and hinge for a physical sheave or spool. | The solver uses declared obstacles and guides. It does not supply free sag, 3D bore-wall contact or arbitrary topology switching. |
| Rolling joint, eyelet or flexible body | Use `geom`, compound proxies or `flex` for body collisions. Specify cable surfaces, guides or eyelets separately. | Collision shape, visual mesh and cable envelope need not match. An STL hole does not guarantee a hole in native mesh collision. |

## MJCF Format, Units and Frames

| Level | Common elements or attributes | Convention |
|---|---|---|
| Model and resources | `<mujoco>`, `compiler`, `option`, `size`, `default/class`, `include`, `asset/mesh`, `asset/material` | Use relative paths. Check `meshdir`, `scale` and the CAD origin. Reuse attributes with `default`; `childclass` can pass defaults down the tree. |
| Bodies and points | `worldbody/body`, `inertial`, `geom`, `site`, `camera` | `body pos` is in the parent frame; its elements use the owning body's local frame. A `site` is not a collision object. Specify `inertial` explicitly when mass and center of mass must be controlled. |
| Joints | `hinge`, `slide`, `ball`, `freejoint`; `axis`, `pos`, `limited/range`, `ref`, `stiffness/springref`, `damping`, `armature`, `frictionloss` | `hinge` and `slide` are scalar. A free body is a direct child of `worldbody`. `armature` is not bearing damping. |
| Cross-references | `tendon`, `equality`, `contact`, `actuator`, `sensor`, `keyframe/key` | Reference meaningful names such as `thumb_tip_anchor` and `winch_hinge`. Use `key` to store initial `qpos`, `qvel`, `ctrl` and other state. |

Use a consistent system of units (m, kg, s, N and N·m) and set `<compiler angle="radian"/>` explicitly in examples. MJCF otherwise defaults to degrees for angular quantities subject to the compiler's conversion. Runtime hinge positions and angular velocities use radians and rad/s. `compiler angle` **does not convert control commands**. Quaternions are ordered `w x y z`. A floating platform occupies seven `qpos` entries and six `qvel` entries; do not integrate quaternion components as translations.

Official references: [MJCF XML](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html), [Modeling and coordinate frames](https://mujoco.readthedocs.io/en/3.4.0/modeling.html#coordinate-frames).

## Cable Routes, Spools and Commands

| Interface | Key fields | Use |
|---|---|---|
| Native `tendon/spatial` | `site/@site`, `geom/@geom`, `sidesite`, `pulley/@divisor` | A via site stays on the route. `sidesite` is a native wrapping reference, not a via point. In 3.4.0 only spheres and cylinders are wrap geoms; consecutive wrap geoms require an intermediate site. |
| Native `tendon/fixed` | `<joint joint="…" coef="…"/>` | Weighted combination of scalar joint coordinates, useful for ideal coupling but not a 3D wrap path. |
| Native tension baseline | `limited`, `range`, `springlength`, `stiffness`, `damping`, `frictionloss` | Set `limited="true" range="0 Lmax"` (replace `Lmax` with a number) for a geometric length limit on the long side. An ordinary spring and damper are not automatically a slack-aware cable model. `frictionloss` is not wrap-angle friction. |
| Native actuation | `motor`, `position`, `velocity`, `general`, `muscle`; `joint/tendon/site`, `gear`, `ctrlrange`, `forcerange` | Depending on type and transmission, `ctrl` can be a force command, a reference or muscle excitation. `ctrlrange` limits input; `forcerange` limits scalar actuator output. |
| MuJoCable routing | `route_mode`, `route_tendon`, `wrap_geoms`; `site user` roles | In `surface` mode, endpoint `1`, initialization hint `2` and physical guide `3` have different meanings. `gear="0"` prevents double loading along the native seed. Its native length is not the plugin's solved length. |
| MuJoCable drive and friction | `target_contraction`, `joint_spool_angle`, `capstan_mu` and related settings | Take-up, shaft angle, bearing damping and cable/sheave friction are different quantities. Check fixed-radius spool signs and units with $\Delta c=\pm r\,\Delta\theta$; the sign depends on winding. |

**Minimal native example.** Download <a href="../downloads/mujoco_native_cable.xml" download>mujoco_native_cable.xml</a>. The cylinder is fixed and the payload slides. `pull` is a unit-gain motor with `gear="-1"`, so positive input tends to shorten the path. This is a force-controlled baseline, **not** a slack-aware winch model.

```{literalinclude} downloads/mujoco_native_cable.xml
:language: xml
:start-at: <tendon>
:end-at: </actuator>
```

Official references: [Native spatial tendon](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#tendon-spatial), [Actuation model](https://mujoco.readthedocs.io/en/3.4.0/computation/index.html#actuation-model). For plugin settings see [Routing](routing.md), [Actuation](actuation.md) and [Friction](friction.md).

**Check tension feasibility for a parallel platform.** For frictionless cables with uniform tension along each route, define the length Jacobian by $\dot{\boldsymbol L}=J_L\boldsymbol v$. The cables then generate $\boldsymbol\tau=-J_L^\mathsf{T}\boldsymbol T$. Check whether the desired generalized force is achievable subject to $0\le T_i\le T_{i,\max}$; rank alone is insufficient. For a straight cable, use `mj_jacSite` at both endpoints and project along the cable direction. With surface routing, use the solved path rather than the derivative of the seed length. When segment friction is present, a single uniform tension cannot replace the segment forces.

## Closed Loops, Contact and Compliance

| Purpose | Interface and format | What to check |
|---|---|---|
| Join two branches at a point | `equality/connect site1="a" site2="b"`, or `body1/body2/anchor` | The site form brings the two points together without aligning orientation. Align them at the start to avoid an assembly transient. |
| Fix the relative pose of two branches | `equality/weld`; use `site1/site2` to align frames, or `body1/body2/relpose` | This is a constraint, not the same as placing parts on one rigid body in the tree. It does not model a rolling bearing. |
| Ideal ratio, fixed length or coordination | `equality/joint joint1="a" joint2="b" polycoef="0 1 0 0 0"`; `equality/tendon tendon1="a" tendon2="b"` | These relate scalar joints or native lengths as deviations from their references. Equalities are bilateral. Constraining the MuJoCable seed does not constrain its solved cable. |
| Contact and exclusions | `geom contype/conaffinity/condim/friction`; `contact/pair geom1/geom2`; `contact/exclude body1/body2` | For dynamic pairs check masks and filters such as common-body and parent-child rules. `pair` defines an explicit geom pair and its settings. The three `geom friction` terms describe sliding, twisting and rolling friction, not a no-slip rolling constraint. |
| Concavity, apertures and compliance | Use compound geoms or suitable `flexcomp/flex`; a low-order beam model can use `joint stiffness/damping`. | Ordinary mesh geom collision is convex even when the STL looks non-convex. Calibrate effective hinge stiffness or verify material and mesh convergence for a flex model. |
| Numerical stability | `option timestep/integrator/solver/iterations/tolerance`; constraint `solref/solimp` | Positive `solref` takes a time constant and damping ratio; `solimp` sets constraint impedance. Check geometry and constraint rank, then test timestep convergence before tuning iterations. |

**Minimal closed-loop example.** Download <a href="../downloads/mujoco_four_bar.xml" download>mujoco_four_bar.xml</a>. Three tree hinges and one point closure form a planar four-bar linkage. Away from singularities, the two independent planar closure equations leave one effective DOF. Set `crank_target` to move the input link, and measure the distance between `data.site("coupler_tip").xpos` and `data.site("rocker_tip").xpos`. A soft constraint can have residual error under load; `model.nv` is not the number of independent mechanism coordinates.

```{literalinclude} downloads/mujoco_four_bar.xml
:language: xml
:start-at: <equality>
:end-at: </equality>
```

Neither model needs a plugin or external asset. From the download folder, run the following on macOS (use `python` in place of `mjpython` on other platforms):

```bash
mjpython -m mujoco.viewer --mjcf=mujoco_native_cable.xml
mjpython -m mujoco.viewer --mjcf=mujoco_four_bar.xml
```

Official references: [Equality](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#equality), [Contact](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#contact), [Constraint solver parameters](https://mujoco.readthedocs.io/en/3.4.0/modeling.html#solver-parameters).

## Python and C Runtime

| Stage | API or data | Read/write convention |
|---|---|---|
| Compile and reset | `MjModel.from_xml_path` (C: `mj_loadXML`), `MjData(model)` (C: `mj_makeData`), `mj_resetData`, `mj_resetDataKeyframe` | Load the required dynamic library with `mj_loadPluginLibrary` before loading a plugin model. After a reset or direct `qpos/qvel` edit, use `mj_forward` to refresh derived quantities. |
| Input and integration | `data.ctrl`, `mj_step`, `mj_step1/mj_step2`, `mjcb_control` | Set controls before stepping. Split stepping can expose current kinematics for feedback but does not retain RK4 integration semantics. |
| External loads | `qfrc_applied`, `xfrc_applied`, `mj_applyFT` | Index by `nv` and body, respectively. `mj_applyFT` adds the generalized effect of a force and torque at a world point to a target vector. Clear loads explicitly when a perturbation ends. |
| Pose, velocity and Jacobians | `mj_name2id`, `jnt_qposadr/jnt_dofadr`, `site_xpos/site_xmat`, `mj_jacSite`, `mj_objectVelocity` | Joint IDs are not state offsets. Jacobians have `nv` columns. `mj_objectVelocity` places angular velocity first and uses `flg_local` to select axis orientation. |
| Contact and loop constraints | `ncon`, `contact`, `mj_contactForce`, `eq_active`, `efc_type/efc_id/efc_pos/efc_force` | `mj_contactForce` returns a six-component wrench in the contact frame. Look up constraint rows by type/ID rather than fixed indices. Enable or disable a closure at `data.eq_active[equality_id]`. |
| Analysis and repeatability | `mj_getState/mj_setState`, `mj_integratePos/mj_differentiatePos`, `mj_inverse`, `mjd_transitionFD`, `mj_mulM` | Log the state bitmask and controls. Use quaternion-aware increments. Check the applicability of local linearization around slack or changing contacts. |
| Model generation and interaction | `MjSpec.from_file`, `spec.compile()`, `mujoco.viewer.launch_passive`, `viewer.sync()` | Generate repeated branches in `MjSpec` or structured XML. Recompile after topology edits. In the passive viewer, the script owns stepping and synchronizes the display. |

```python
import mujoco

m = mujoco.MjModel.from_xml_path("mujoco_native_cable.xml")
d = mujoco.MjData(m)
d.actuator("pull").ctrl[0] = 0.75
for _ in range(20):
    mujoco.mj_step(m, d)

mujoco.mj_forward(m, d)  # Recompute observations at the final state.
length_m = float(d.sensor("cable_length").data[0])
rate_m_s = float(d.sensor("cable_rate").data[0])
pull_N = float(d.sensor("pull_output").data[0])
print(f"L={length_m:.6f} m, dL/dt={rate_m_s:.6f} m/s, motor={pull_N:.3f} N")
```

Official references: [Python API and named access](https://mujoco.readthedocs.io/en/3.4.0/python.html#named-access), [C API Functions](https://mujoco.readthedocs.io/en/3.4.0/APIreference/APIfunctions.html), [C API Types](https://mujoco.readthedocs.io/en/3.4.0/APIreference/APItypes.html).

## What to Log

| Question | Observable | Interpretation |
|---|---|---|
| Did the motor take up cable as intended? | `ctrl`, spool `jointpos/jointvel`, `actuatorfrc` or `data.actuator_force` | Log command units, gear and output sign. Scalar actuator output is not necessarily joint torque or every segment's tension. |
| Are length, speed and slack correct? | Native `tendonpos/tendonvel` or `data.ten_length/ten_velocity`; plugin `sensor/plugin` | For MuJoCable, log solved length, free length, tension, `taut` and route status. Read `sensor_dim` rather than hard-coding offsets into the whole `sensordata` vector. |
| Is the loop closed? Does anything slip or penetrate? | World positions of both sites, `efc_pos/efc_force`, `contact`, `mj_contactForce` | Label the units, frames and sample phase. Check relative tangential velocity at a contact for rolling. Keep loop-closure and cable-route residuals separate. |
| What are the end-effector pose, grasp loads or base reactions? | `framepos/framequat`, `framelinvel/frameangvel`, `touch`, `force/torque` | `touch` aggregates normal contact force in a region; `force/torque` measures a parent/child body interaction, not a particular geom pair. |

For a hardware or baseline comparison, match geometry, inertias, initial poses, command units, pretension and sampling times before changing the route or friction model. For a parallel platform, record every cable tension and the task-space wrench; check configurations near slack or singularities separately. Record the MuJoCo, model and plugin versions and `timestep`; store copies of array data when logging snapshots.

Official references: [Sensors](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#sensor), [Simulation](https://mujoco.readthedocs.io/en/3.4.0/programming/simulation.html). See MuJoCable's [Sensors](sensors.md) and [Architecture](architecture.md) for plugin data and callbacks, and MuJoCo's [Engine plugins](https://mujoco.readthedocs.io/en/3.4.0/programming/extension.html#engine-plugins) for the host interface.
