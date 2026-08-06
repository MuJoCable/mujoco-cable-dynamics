# MuJoCable Modeling and Threading Guide

## Separate the Three Modeling Layers

1. **MuJoCo mechanics:** bodies, joints, inertia, collision geometry, ground,
   and self-collision. The plugin does not replace rigid-body contact.
2. **Route geometry:** endpoints, initialization hints, persistent guides, and
   cylinder or mesh envelopes.
3. **Cable dynamics:** free length, stiffness, damping, slack, pretension,
   tension limit, spool/control state, friction, and sensors.

Stabilize the contact-only mechanism first, add routes second, and tune
pretension and control last.

## Site Roles

With `<size nuser_site="1"/>`, the first site user value means:

| Value | Role | Runtime behavior |
|---:|---|---|
| 1 | endpoint | Physical cable end and force node |
| 2 | route hint | Selects wrap order and side at initialization only |
| 3 | guide/eyelet | Persistent physical route node with optional friction |
| 0 | ordinary site | No plugin semantics |

Keep the seed tendon nearly invisible (`width="0.000000001"`). MuJoCo draws
that tendon as straight segments through every site; the plugin scene callback
draws the solved runtime route.

## Choose the Route by Mechanism

### Straight or fixed-guide route

Use `route_mode="native"` for tensegrity and fixed guide paths (Demos 17-19).
Prefer a native MuJoCo tendon when no unilateral free-length or plugin state is
needed.

### Cylinder pulley

Use `route_mode="surface"`, one role-2 hint per cylinder, and a matching
`wrap_geoms` entry (Demo 15). Place endpoints outside the cylinder and put
the hint on the intended wrap side; it is not a hand-authored tangent point.

```xml
<spatial name="route_seed" width="0.000000001">
  <site site="start"/><site site="upper_hint"/><site site="end"/>
</spatial>
<config key="route_mode" value="surface"/>
<config key="route_tendon" value="route_seed"/>
<config key="wrap_geoms" value="pulley_wrap"/>
```

List multiple hints and geoms in threading order. Demo 10 uses a compound
surface route and common-tangent bridges rather than fixed hint corners.

### Closed mesh envelope

- `convex_surface`: closed convex proxy; most robust (Demo 14).
- `taut_obstacle`: shortest nonpenetrating route over a closed nonconvex
  obstacle, allowed to bridge concavities.
- `guided_surface`: homotopy-guided corridor for ligament-style surface
  routes (Demo 25).

Meshes must be closed, manifold, consistently oriented, and free of
self-intersection. Visual, collision, and route meshes may be distinct, but
their roles must be documented. Start with a low-resolution proxy before using
the extracted physical exterior.

### Through-holes and eyelets

STL/OBJ preserves a geometric hole, but ordinary rigid mesh collision is often
convexified. Use rigid flex for true nonconvex aperture collision (Demo 31).
The present massless centerline cable does not discover and enter holes
automatically: add role-3 guides at the mouth/exit to define threading topology
(Demos 28, 32, and 33).

```xml
<site name="eyelet" user="3" .../>
<config key="guide_friction_mu" value="0.10"/>
<config key="capstan_direction" value="forward"/>
```

A role-3 guide is a reduced-order point eyelet. It does not model bore radius,
distributed wall contact, jamming, or wear.

### Passive ligament and rolling joint

Omit the actuator for a passive cable. Finite stiffness and pretension create a
restoring ligament that resists extension but never pushes. Rolling motion is
produced jointly by surface contact and unilateral ligaments; the plugin does
not create an implicit hinge. See Demos 13, 16, and 20.

### Antagonistic routing and reserve

The opposite route normally lengthens while one side contracts. Give each side
reserve `R` so

\[
L_{free}=L_{home}+R-c.
\]

Set `pretension_offset="-R"`, take up both sides to `R`, then command
`c_+=R+u` and `c_-=R-u`. Demo 33 uses `R=25 mm`; the release-side sensor
must return to zero tension at maximum motion.

### Winch and free pulley

Use spool joint/radius/direction/reserve for a winch. A free pulley needs a
physical hinge and inertia. Demo 29 validates `tau=R(T2-T1)`, rope/rim speed,
and frictional dissipation. Use velocity-directed friction if sliding may
reverse.

## Material Law and Tuning

The unilateral Kelvin-Voigt law is

\[
T=\operatorname{clip}\left(k[L-L_{free}-s]_+ + c\dot L,0,T_{max}\right).
\]

Estimate stiffness from `EA/L`; Poisson ratio is not a separate parameter in
this one-dimensional law. Identify damping from decay or tension-step data.
Use `slack` as a small deadband, not as a substitute for spool reserve.
Near-inextensible cables still require finite stiffness, time-step convergence,
and possibly the implicit-compliance mode.

## Debugging Checklist

1. Hide the seed tendon and distinguish endpoints, hints, and guides.
2. Validate joints and collision without cable forces.
3. Run with `--show-route-debug --show-cable-state` and inspect residuals.
4. Sweep the full pose range with low stiffness and pretension.
5. Check every free segment against every wrap obstacle and inspect tangent
   continuity at moving-surface interfaces.
6. Increase stiffness, damping, pretension, and control rate gradually.
7. Validate virtual work, static equilibrium, Capstan ratios, and time-step
   convergence.
8. Add self-collision and ground contact last. A gray cable is slack by design;
   a massless cable does not sag.

See [RUN_ALL_DEMOS.md](RUN_ALL_DEMOS.md) for executable commands.
