# MJCF Model Structure

## Three Separate Layers

| Layer | Definitions | Solver |
|---|---|---|
| Mechanism | Bodies, joints/freejoints, mass, inertia, collision geom/flex and ground | MuJoCo |
| Route | Endpoints, hints, guides, seed and `wrap_geoms` | MuJoCo for native; MuJoCable for surface |
| Transmission | Free length, stiffness, damping, slack, pretension, control and friction | MuJoCable law and force map |

Check mechanism contacts without cables first, then add one cable. **The plugin neither creates a rolling joint automatically nor removes an existing hinge.** Contact-defined rolling and virtual-hinge approximations require different mechanisms, not merely different actuator names.

## One Instance per Cable

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

This fragment needs matching body/site/tendon definitions. Use an [example ZIP](gallery.md) or the [minimal model](quickstart.md) as a complete starting point.

## Site Roles

| First `site user` slot | Role | Mandatory at runtime | Affects length/force | Placement |
|---:|---|:---:|:---:|---|
| 1 | Physical endpoint | Yes | Yes | Real attachment at either seed end; avoid embedding inside a solid |
| 2 | Initialization hint | No | No | Intended side, one per wrap entry |
| 3 | Physical guide | Yes | Yes | Real eyelet, ring or conduit exit, not an unsupported point |
| 0 | Ordinary marker | No | No | Visual/measurement reference; omit from a surface seed |

Site `pos` is relative to its parent body. Recheck alignment after parent-body motion, orientation changes or mesh compilation transforms.

```xml
<tendon>
  <spatial name="flexor_seed" width="0.000000001" rgba="0.9 0.15 0.1 1">
    <site site="prox_end"/><site site="prox_hint"/>
    <site site="dist_hint"/><site site="dist_end"/>
  </spatial>
</tendon>
```

Hint order must match `wrap_geoms="prox_wrap dist_wrap"`. A geom may repeat for multiple visits. Do not add seed springs, damping or length constraints: they would act along the native polyline, not the solved surface route.

## Active and Passive Cables

An active surface cable adds a plugin actuator referencing the same seed, with `gear="0"`. A passive surface cable keeps the instance and sensor but omits the actuator. Its contraction is zero without input, yet passive-stage forces remain active.

No actuator does not mean constant length. For a compliant joint, check stiffness, slack, pretension, contact and remaining degrees of freedom together. Start at moderate stiffness to verify mobility before calibration.

## Geometry Responsibilities

| Geometry | Purpose | Common mistake |
|---|---|---|
| Visual mesh | Appearance | Assuming a visible hole is also a traversable collision hole |
| Collision geom / rigid flex | Solid-body contact | Losing holes and concavities through convexification |
| Route geom | Obstacle for the configured cable | Misalignment with visuals or omitted obstacles |

Shapes may be shared or simplified separately, but align them and document approximation errors. Elastic hinges in finger and log-spiral models are reduced-order beam models, not continuum finite elements.

## Material Parameters

For a uniform slender cable at small axial strain, estimate $k=EA/L_0$ in N/m. Young's modulus $E$ in Pa is not itself `stiffness`. Poisson's ratio is not an input to this centerline model. Damping, braid effects, effective area and attachment compliance require calibration.

Making $k$ arbitrarily large does not safely create an inextensible cable. Check timestep convergence, mass and energy. Exact length constraints have different numerical semantics from finite stiffness.
