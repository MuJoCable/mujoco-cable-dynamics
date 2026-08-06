# MuJoCable: a surface-routed unilateral cable plugin for MuJoCo

**Technical note for upstream architecture discussion (draft, 2026-07-22)**

The full implementation report, including the MuJoCo/MuJoCable ownership
boundary and UML diagrams, is available in
[`plugin_method_strategy_report.md`](plugin_method_strategy_report.md).

> This note describes a research beta, not a proposed merge-ready MuJoCo
> component. Values under **current evidence** come from checked-in tests or
> retained experiment artifacts. Items marked **required before outreach** have
> not yet been demonstrated systematically.

## 1. Problem and scope

MuJoCo spatial tendons are efficient scalar-length elements for actuation,
limits, equality constraints, spring-dampers, friction loss, and pulley
branches. Their geometry is intentionally restricted. The
[MuJoCo overview](https://mujoco.readthedocs.io/en/stable/overview.html#tendons)
states that only sphere and cylinder wrapping is supported, cylinders are
treated as infinite for wrapping, and multiple wrapping geoms must be separated
by sites to avoid an iterative solver. This closed-form design is valuable and
should remain the default.

Some cable-driven mechanisms need a different tradeoff. Examples include a
cable running over a noncircular pulley, a ligament crossing two moving rolling
surfaces, or a tendon path around a closed mesh proxy. In these cases a fixed
intermediate site changes the physical route. In particular, for surfaces
`S1(q)` and `S2(q)`, the route segment between them should be determined jointly
with the two contact curves:

```text
minimize    length(Gamma(q))
subject to  Gamma does not enter any obstacle,
            contact nodes lie on their assigned surfaces,
            the initialized route topology is preserved.
```

At a smooth optimum, the free segment leaves `S1` and reaches `S2` tangentially.
When the gap tends to zero, the solver must preserve a continuous compound
envelope rather than introduce a fixed point, a kink, or an arbitrary side
switch.

This limitation has appeared in community usage. In
[MuJoCo issue #511](https://github.com/google-deepmind/mujoco/issues/511), a user
asked for cable interaction with robot meshes and noted that two wrapping geoms
could not be traversed without an intermediate site. The maintainer response
clarified that tendons do not generally collide and that the restricted wrap
set is required for analytic speed. This proposal does not challenge that
choice. It asks how an iterative, opt-in route solver should integrate through
the plugin API.

The search performed for this draft did not identify a dedicated MuJoCo issue
for rolling-contact joints. Rolling-joint examples are therefore motivation for
the general route interface, not evidence that MuJoCo has promised a particular
rolling-joint feature.

### Dynamics boundary

MuJoCable models a **massless, axial, tension-only cable**. It is intended for
quasi-static and low-frequency cable-driven rigid mechanisms. It currently does
not model:

- distributed cable mass, gravity sag, transverse vibration, or wave travel;
- bending/torsional stiffness, cable-cable self-contact, or multilayer winding;
- a full stick--slip material-velocity constraint between cable and surface;
- automatic global switching between route homotopy classes;
- guaranteed global optimality on arbitrary nonconvex or self-intersecting
  surfaces.

Consequently, it should not be described as a general rope simulator or a
hardware-validated rolling-contact digital twin.

## 2. Method

### 2.1 Route representation

An MJCF seed tendon declares the ordered endpoints and initialization hints.
Endpoints and explicit guides are physical nodes. Role-2 hints select a wrap
surface, side, and initial triangle corridor, then cease to be route nodes. The
runtime path contains endpoints, active surface contacts, and real guides only.

The current route backends are:

- **native**: read MuJoCo tendon length, velocity, and transmission geometry;
- **cylinder**: optimize entry/exit axial coordinates and unwrapped angular
  branch, yielding straight tangents plus a helical geodesic;
- **convex surface**: unfold and optimize a selected triangle strip on a closed
  convex mesh;
- **taut nonconvex obstacle**: maintain an active, nonpenetrating path on a
  closed oriented mesh and remove contacts that would require adhesive normal
  force;
- **guided surface**: preserve an initialized support corridor while allowing
  runtime contact points to move independently of the hints;
- **compound surface**: jointly adjust the exit of one surface and entry of the
  next to obtain a finite common-tangent bridge, with a near-zero-gap merge
  rule.

Mesh initialization checks closedness, manifold adjacency, orientation,
degenerate triangles, and nonadjacent triangle intersections. Runtime route
state is warm-started and retains its initialized topology. A failed route
produces no cable force for that step, preserves the previous warm start, and
reports a diagnostic state.

### 2.2 Cable law and force mapping

Let `L` be the solved route length and `c` the commanded contraction. The free
length and unilateral extension are

```text
L_free = L_home - c - pretension_offset,
e      = L - L_free - slack.
```

For the basic Kelvin--Voigt option,

```text
T = clamp(max(0, k e + d Ldot), 0, T_max).
```

A smooth taut transition and activation hysteresis are available to reduce
chatter at `e = 0`. A spool can map angle to signed stored length and contributes
the reaction torque

```text
tau_spool = -T dc/dtheta.
```

For an impending-slip Capstan model, segment tension follows the
Euler--Eytelwein relation

```text
T_high / T_low = exp(mu theta).
```

With no friction, all segments share one tension. Each path node receives the
vector sum of its adjacent segment tensions. Forces and moments are mapped to
the owning rigid bodies with `mj_applyFT`. For a frictionless scalar cable this
is checked against virtual work:

```text
qfrc ~= -T dL/dq.
```

The route derivative uses current endpoint/contact body velocities and the
envelope theorem; initialization hints do not contribute velocity or force.

## 3. MuJoCo integration

The plugin follows the documented
[engine-plugin model](https://mujoco.readthedocs.io/en/stable/programming/extension.html#engine-plugins).
MuJoCo currently supports actuator, sensor, passive-force, and SDF plugin
capabilities, including one-to-many use of an instance. MuJoCable registers one
instance with actuator, passive-force, and sensor capabilities:

| Pipeline role | Current behavior |
|---|---|
| Passive force | Solve/update the surface route, evaluate cable state, and add route-node forces to `qfrc_passive`. A passive ligament has no actuator. |
| Actuator | Accept contraction/spool commands and report scalar cable tension through `actuator_force`. Surface-mode transmission gear is zero to avoid duplicate native tendon forces. |
| Sensor | Report `L`, `Ldot`, free length, contraction, extension, tension, taut/saturated flags, route status, tangency residual, surface residual, and iterations. |
| State/data | Store physical filtering/hysteresis state in `plugin_state`; reconstruct caches and geometry accelerators as plugin data. |
| Visualization | Draw the same solved runtime polyline used for length and force, independently of the hidden seed tendon. |

The standalone beta uses only public plugin capabilities for the explicit/local
path. An experimental implicit compliant operator currently requires a small
MuJoCo source-tree extension. It should not be presented as part of the portable
plugin until an upstream-supported passive-force Jacobian/operator hook exists.

## 4. Validation status and publication gate

### 4.1 Current evidence

The following results exist in the development artifacts. They are useful
engineering evidence, but a public artifact bundle must regenerate them from a
clean checkout before outreach.

| Check | Current result | Interpretation |
|---|---:|---|
| Portable beta regression | `56/56` tests passed on a clean local MuJoCo 3.4.0 build | Current macOS result; public Linux/macOS CI is still required. |
| Analytic single-cylinder length | absolute error `9.30e-7 m` | Below the current `1e-5 m` acceptance threshold for one sampled geometry. |
| Analytic double-cylinder length | absolute error `9.12e-7 m` | Point validation only; not yet a broad pose/radius scan. |
| Cylinder tangency residual | `4.75e-8` maximum in the acceptance artifact | Route is locally tangent in the tested cases. |
| Cylinder surface residual | `0 m` in the acceptance artifact | Contact points lie on the analytic surface in the tested cases. |
| Virtual work | `1.43e-5 N` absolute generalized-force error for `T = 2 N` | Approximately `7.4e-6` relative to the expected force in one finite-difference check. |
| Slack cable | zero segment tension and zero resultant force | Confirms no compressive push in the tested state. |
| Capstan ratio | `1.181130106` measured vs. `1.181130646` expected | Relative error about `4.6e-7` in one plugin acceptance case. |
| Force/moment balance | `4.74e-16 N`, `2.78e-17 N m` residual in the frictionless case | Discrete node-force mapping is balanced in the tested route. |
| Surface pulley | maximum surface error `1.30e-8 m`; valid route during a `60 mm` lift command | Stable minimal visual/force example for the sampled rollout. |
| Native/plugin taut comparison | max cross-tension difference `0.0261 N`; shape difference `8.74e-5 m`; node RMSE `6.93e-5 m` | Agreement in one matched tensegrity regime, not general equivalence. |
| Performance, matched tensegrity | `10.98 us/step` plugin vs. `6.33 us/step` native (`1.73x`) | One machine and one model only; no scaling conclusion. |
| Faive PIP research case | `32.2 deg`, contact fraction `0.999`, p95 `11.96 ms/step` for a `6 mm` command | Demonstrates coupled contact/cable motion, but not the nominal `51.6 deg` range or hardware accuracy. |
| 100_fingers hand integration | `20 DOF`, `25` surface cables, finite coordinated motion; about `0.07` real-time factor in one `0.3 s` macOS rollout | Integration smoke test only; systematic scaling and anatomical validation remain open. |

The current faceted Faive two-surface bridge can reach approximately `21.2 deg`
of interface tangent mismatch in its regression envelope. That result is an
open solver limitation, not a successful common-tangent validation.

### 4.2 Required before outreach

The public beta should include the following preregistered matrix and retain raw
CSV/JSON data, configuration, commit, MuJoCo version, compiler, platform, and
random seeds.

| Validation | Minimum protocol | Acceptance/reporting requirement | Status |
|---|---|---|---|
| Analytic cylinders | Sweep radius, endpoint distance, axial offset, wrap side, relative pose, and one/two cylinders. | Report P50/P95/P99/max `L`, `Ldot`, and Jacobian errors; target P95 length error `<1e-5 m`. | Partial |
| Virtual work | Central differences for every affected DOF over multiple perturbation sizes. | `qfrc ~= -T dL/dq`; target relative error `<1%` away from topology switches. | Partial |
| Slack no-push | Sweep extension and velocity through the slack/taut transition. | No positive/compressive cable work while slack; report hysteresis behavior. | Unit/point tests only |
| Capstan | Sweep `mu`, wrap angle, direction, and reversal. | Compare `T_high/T_low` with `exp(mu theta)` and state the impending-slip assumption. | Partial |
| Route no penetration | Random SO(3) scans on convex and nonconvex closed meshes and two-surface routes. | Zero penetrating free segments; report invalid/degraded rate and worst residual. | Partial |
| Timestep convergence | At least `0.1, 0.2, 0.5, 1, 2 ms`, with stiffness/damping sweeps. | Converged motion/tension curves, stability domain, energy and failure statistics. | Missing |
| Scaling | Meshes from roughly `32` to `4096` faces and increasing cable count. | Median/P95 route and step time, memory, iterations, and invalid rate versus problem size. | Missing |
| Native/high-fidelity baselines | Matched straight, single-pulley, dual-pulley, and tensegrity scenes. | Accuracy and state/time tradeoffs; no claim outside the massless low-frequency domain. | Native partial; high-fidelity missing |
| Hardware pulley/winch | Encoder drum, known loads, two tension sensors, several speeds/frictions, held-out tests. | Compare length, load motion, both-side tension, and Capstan ratio; target errors defined before data collection. | Missing |
| Minimal examples | One stable single pulley and one stable dual/compound pulley, each compared numerically with theory. | One-command reproduction on Linux and macOS. | Models exist; clean public artifact pending |

Repository readiness is also part of technical credibility. Before contacting
the MuJoCo team, the project must have a public sanitized repository, tagged
Release, license and third-party notices, minimal MJCFs, passing cross-platform
CI, checksums, and a reproducible experiment entry point. A private repository
does not allow maintainers to evaluate architecture, tests, provenance, or
performance.

## 5. Requested upstream guidance

The request should remain narrower than “merge this plugin.” The design
questions for MuJoCo maintainers are:

1. **Custom route interface.** Would a plugin capability that returns tendon
   length, velocity/Jacobian, route points, and optional segment metadata fit
   MuJoCo's architecture, or should custom cables remain passive-force plugins?
2. **Implicit passive forces.** Is a matrix-free or Jacobian contribution from
   passive-force plugins desirable? A supported hook would avoid a source-tree
   patch for stiff cable stabilization.
3. **Geometry queries.** Which mesh topology, BVH, ray/segment, or signed-distance
   APIs are intended to be stable for plugin-owned geometric solvers?
4. **Single source of truth.** Can custom route geometry be exposed to standard
   visualization and sensors so displayed cable, scalar length, Jacobian, and
   applied force are guaranteed to represent the same path?
5. **Contribution path.** Would the team prefer an external-plugin example,
   one or more small generic API PRs, or a first-party plugin only after broader
   validation and maintenance commitments?

The recommended sequence is: public beta and reproducible core validation;
then a Feature Request/pre-RFC issue; then small self-contained interface PRs if
the maintainers consider them useful. The nonconvex and multi-surface algorithms
should remain external until their topology continuity, performance, and
failure behavior are well characterized.

## 6. References

1. MuJoCo, [Contributing guide](https://github.com/google-deepmind/mujoco/blob/main/CONTRIBUTING.md).
2. MuJoCo, [Overview: Tendon geometry and engine plugins](https://mujoco.readthedocs.io/en/stable/overview.html).
3. MuJoCo, [Extensions: engine plugins](https://mujoco.readthedocs.io/en/stable/programming/extension.html#engine-plugins).
4. MuJoCo issue [#511: How to make tendon have collision attribute?](https://github.com/google-deepmind/mujoco/issues/511).
5. MuJoCo issue [#1336: How to route a Cable in MuJoCo?](https://github.com/google-deepmind/mujoco/issues/1336).
6. Euler--Eytelwein/Capstan relation, used here only as an impending-slip segment-tension model.
