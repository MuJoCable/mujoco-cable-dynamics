# MuJoCable Method and Implementation Report

Version: 2026-08-06

## Abstract

MuJoCable implements `mujoco.cable.unilateral`, a physics-informed,
massless-cable surrogate for pulleys, winches, tensegrity, rolling-contact
joints, hands, and routed compliant robots. It is not a deformable-rope solver:
cable mass, sag, bending, torsion, waves, and self-contact remain outside its
state. MuJoCo continues to own rigid-body dynamics, contact, constraints, and
time integration.

The plugin preserves `route_mode="native"` and adds runtime surface routes
over arbitrarily oriented cylinders and closed convex or guided nonconvex mesh
obstacles. Initialization hints select wrap order and homotopy, but runtime
length, velocity, contact nodes, tension, and body forces use the solved route.
The constitutive model is unilateral, supports controlled free length and
winch reserve, and optionally propagates segment tension using the
Euler-Eytelwein relation. Persistent role-3 guides provide a reduced-order
eyelet model with local friction. The latest release adds a dual-reserve,
antagonistic log-spiral robot benchmark with module self-collision and a
five-coefficient friction sweep.

## 1. Scope and MuJoCo Boundary

Native MuJoCo spatial tendons are preferred for routes explicitly defined by
sites and supported wrap objects. MuJoCable targets mechanisms that also need:

1. initialization hints that do not remain fixed force nodes;
2. tangent/geodesic routes on moving obstacle surfaces;
3. unilateral slack with controlled free length;
4. direct surface-node force application without a second seed-tendon force;
5. route validity, residual, tension, and free-length diagnostics.

The division of responsibility is:

| Layer | Owner | State and computation |
|---|---|---|
| Rigid mechanics | MuJoCo | bodies, joints, inertia, collision, constraints, integration |
| Route geometry | MuJoCable | tangent points, surface corridors, common-tangent bridges |
| Cable dynamics | MuJoCable | free length, slack, tension, friction, spool state |
| Rendering | MuJoCo scene + plugin callback | runtime cable polyline and state color |

![Component boundary](figures/mujocable_component_uml.png){width=60%}

The plugin writes surface-route forces into `qfrc_passive`. A surface actuator
keeps MuJoCo's control and force-reporting interface but must use `gear="0"`,
preventing the force-disabled seed tendon from applying a duplicate
transmission force.

## 2. MJCF Interface and Route Roles

The first site user field defines:

- `user="1"`: physical endpoint;
- `user="2"`: initialization hint, ignored as a runtime force node;
- `user="3"`: persistent physical guide/eyelet;
- `user="0"`: no plugin role.

```xml
<size nuser_site="1"/>
<tendon>
  <spatial name="route_seed" width="0.000000001">
    <site site="start"/><site site="proximal_hint"/>
    <site site="distal_hint"/><site site="end"/>
  </spatial>
</tendon>
<config key="route_mode" value="surface"/>
<config key="mesh_route_mode" value="guided_surface"/>
<config key="route_tendon" value="route_seed"/>
<config key="wrap_geoms" value="proximal_surface distal_surface"/>
```

Every role-2 hint must correspond to one `wrap_geoms` entry. The same obstacle
may appear multiple times. A role-3 guide requires no wrap geom and remains a
runtime force node.

The 12-value surface sensor returns

```text
length, velocity, free_length, contraction, extension, tension,
taut, saturated, route_status, tangent_residual,
surface_residual, solver_iterations.
```

## 3. Route Solvers

### 3.1 Native backend

`NativeTendonRoute` reads MuJoCo's compiled spatial-tendon length, velocity,
wrap points, and wrap ownership. The plugin adds unilateral constitutive and
free-length control while retaining the native route.

### 3.2 Cylinder envelope

A cylinder is transformed to local coordinates. Entry and exit tangency,
axial locations, and angular branch are optimized jointly. Unrolling the
cylinder gives a straight geodesic, so a helical surface segment has length

\[
L_s=\sqrt{R^2\Delta\phi^2+\Delta z^2}.
\]

The total route combines endpoint tangent segments and the surface geodesic.
The hint selects the angular branch; it does not appear in the final length.

### 3.3 Convex and guided nonconvex mesh routes

For a closed convex mesh, the hint selects a seed face and a triangle corridor.
The strip is unfolded and edge-crossing coordinates are optimized for
tangential continuity and minimum length.

For a closed nonconvex obstacle, the solver adds mesh adjacency and BVH
visibility tests. A taut-obstacle route adds contact where a free segment
penetrates and removes contacts requiring adhesive normal force, allowing a
taut cable to bridge a concavity. `guided_surface` preserves an initialized
surface corridor for ligament-style routing. Multiple moving surfaces share a
global length objective and a finite common-tangent bridge.

Meshes must be closed, manifold, consistently oriented, and non-self-
intersecting. The current solver preserves the initialized topology and does
not perform arbitrary side switching.

## 4. Cable Constitutive Law and Forces

For route length `L`, controlled contraction `c`, home length `L0`,
pretension offset `p`, and deadband `s`,

\[
L_{free}=L_0-p-c,
\qquad
e=L-L_{free}-s.
\]

The unilateral Kelvin-Voigt law is

\[
T=\operatorname{clip}\left(k[e]_+ + d\dot L,0,T_{max}\right).
\]

Slack cables therefore cannot push. Stiffness may be initialized from `EA/L`;
Poisson ratio is not an independent parameter in this one-dimensional model.

At each runtime node `x_i`, adjacent segment directions give

\[
f_i=T_{i-1}\hat t_{i-1}-T_i\hat t_i.
\]

The plugin applies each force at its world point using `mj_applyFT`.
Virtual-work validation checks

\[
q_{cable}\approx -T\frac{\partial L}{\partial q}.
\]

### Winch reaction

If contraction is coupled to a physical spool angle, enabling reaction torque
adds

\[
\tau_{spool}=-T\frac{dc}{d\theta}.
\]

For two radii on a common shaft, this recovers the quasi-static wheel-and-axle
balance `T_in R_in=T_out R_out`.

## 5. Friction and Eyelets

For wrap angle `theta`, the Euler-Eytelwein relation is

\[
\frac{T_{high}}{T_{low}}=\exp(\mu\theta).
\]

Surface nodes and role-3 guides propagate segment tension within the same
force path. A guide-local forward relation is

\[
T_{out}=T_{in}\exp(-\mu_g\theta_g).
\]

Velocity-directed surface friction uses the regularization

\[
\frac{T_{out}}{T_{in}}
=\exp\left(\mu\theta\tanh(v_{slip}/v_0)\right),
\]

so the direction reverses continuously with relative cable/surface sliding.
This remains a kinetic-friction approximation without static-friction history,
distributed bore contact, wear, or exact no-slip constraints.

## 6. Antagonistic Reserve and Log-Spiral Routing

When one side of an antagonistic mechanism contracts, the opposite route
usually lengthens. A fixed opposite spool then creates unwanted antagonistic
tension. With reserve `R`,

\[
L_{free}=L_{home}+R-c.
\]

In MJCF, `pretension_offset=-R` implements the reserve. Demo 33 first takes up
both sides and then performs differential payout:

\[
(c_+,c_-):(0,0)\rightarrow(R,R)
\rightarrow(R+u,R-u),\quad 0\le u\le R.
\]

The log-spiral geometry defines twelve progressively scaled modules. MuJoCo
owns eleven hinges and module collision; MuJoCable owns the two threaded
role-3 routes, free length, unilateral tension, and eyelet friction. Eleven
explicit adjacent contact pairs supplement ordinary mesh collision.

For `R=25 mm`, active/release commands of `50/0 mm` produced zero final
release-side tension and approximately `176.8 deg` net bend. The very small
active tension (`0.061 N`) reveals under-calibrated hinge stiffness, so this
is a route/control validation rather than a hardware-accurate prediction.

| Eyelet friction `mu` | Net bend (deg) | Active tension (N) | Release tension (N) |
|---:|---:|---:|---:|
| 0 | 177.54 | 0.0588 | 0 |
| 0.015 | 176.83 | 0.0612 | 0 |
| 0.05 | 175.24 | 0.0677 | 0 |
| 0.10 | 173.10 | 0.0802 | 0 |
| 0.20 | 168.88 | 0.1257 | 0 |

![Dual-reserve friction sweep](figures/demo33_dual_reserve_friction_sweep.png){width=88%}

## 7. Numerical Stabilization

Control filtering, maximum contraction rate, taut/slack hysteresis, route
hysteresis, and visual smoothing reduce switching jitter without changing the
static law. The optional matrix-free implicit-compliance mode linearizes a
fixed route and taut active set:

\[
A=M+\sum_i(hd_i+h^2k_i)G_i^TG_i.
\]

It improves high-stiffness cable oscillation but cannot remove changes caused
by route-corridor or contact-active-set switching; those require geometric and
contact hysteresis.

## 8. Validation Summary

- single- and dual-cylinder length errors: below `1e-6 m`;
- cylinder tangent residuals: below `5e-8`;
- virtual-work absolute error: `1.43e-5 N`;
- frictionless endpoint tension ratio: `1.0`;
- `mu=0.35` measured/discrete Capstan ratios: `1.1811301/1.1811306`;
- slack endpoint force: `0 N`;
- wheel-and-axle theoretical/measured tension ratios: `3.000/2.995`;
- free inertial pulley peak angular speed: `8.65 rad/s`;
- free-pulley regularized Capstan p95 relative error: `2.69e-4`;
- Demo 33 initial/maximum module contacts: `4/4`;
- Demo 33 maximum numerical penetration: approximately `4.2e-9 m`;
- all five Demo 33 friction cases ended with zero release-side tension.

These are simulation verification results, not experimental identification.
Hardware validation should measure route shape, both-side tension, spool angle,
payload motion, and energy loss over held-out speeds and friction conditions.

## 9. Limitations

1. The cable is massless and has no sag, bending, torsion, waves, cable-cable
   contact, or material transport.
2. Mesh routes retain initialized homotopy and do not globally change sides.
3. Finite-cylinder end-face transitions are not implemented.
4. Guided eyelets are point models without bore geometry, jamming, or wear.
5. Capstan friction is regularized sliding friction without stick-slip history.
6. Surface forces use `qfrc_passive`; actuator transmission is retained only
   for control and force reporting.
7. Robot mesh self-collision is a MuJoCo MJCF responsibility, not a cable
   plugin capability.
8. The models are physics-informed mechanism surrogates, not validated digital
   twins unless their geometry and material parameters are experimentally
   identified.

## References

[1] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for
model-based control," IROS, 2012.

[2] MuJoCo Documentation, "Extension plugins," "XML reference," and
"Computation." https://mujoco.readthedocs.io/

[3] J. S. B. Mitchell, D. M. Mount, and C. H. Papadimitriou, "The discrete
geodesic problem," SIAM Journal on Computing, 1987.

[4] M. P. do Carmo, *Differential Geometry of Curves and Surfaces*,
Prentice-Hall, 1976.

[5] Euler-Eytelwein capstan relation.

[6] E. Halilaj et al., "A thumb carpometacarpal joint coordinate system based
on articular surface geometry," Journal of Biomechanics, 2013.

[7] K. Gilday et al., "100 fingers: An open source multi-tool hand
prosthesis," Science Robotics, 2025.

[8] Z. Wang, "OpenSpiRobs: Open Spiral Robots Toolkit," 2026.
https://github.com/ZhanchiWang/Open-Spiral-Robots
