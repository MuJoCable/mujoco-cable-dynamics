# Limitations

## Capabilities Not Claimed

- Not a massive cable or flexible rod: no sag, cable inertia, bending/torsion, traveling waves or knots.
- Not general scene-wide cable collision: only configured route surfaces and guides are modeled.
- Not automatic topology planning: no general branch crossing, lift-off/re-engagement, hole search or rope self-contact.
- Not exact static friction: fixed-direction or velocity-regularized Capstan propagation is not a stick complementarity solver.
- Not a GPU plugin: state and iteration do not automatically transfer to MJX/Warp.
- Not a perfectly rigid cable: finite stiffness differs from an exact length constraint.
- Not a fully calibrated digital twin: material, eyelet, damping, contact and pretension parameters still need identification.

## Case-Specific Interpretations

| Case | What it demonstrates | What it does not establish |
|---|---|---|
| Winch radius ratio | Circumferential travel under an explicit ideal equality | Friction alone producing exact no slip |
| Moving pulley | Supporting spans, approximately 2:1 travel and force transfer | A mandatory 2:1 pulley-angle ratio |
| Atwood landing | Ideal pre-impact comparison, then contact/slack | A collision-free closed form valid after impact |
| Faive joint | Different mappings from virtual-hinge and cable inputs | Identical physical meaning for identical numerical inputs |
| SpiRobs friction sweep | Sensitivity of source load and joint distribution | Material/friction identification from simulated shape alone |
| Eyelet guide | Known threading topology and turning loss | Hole diameter, wall pressure, jamming or wear |
| ICRA letters | Mid-plane routing and passive transmission over noncircular profiles | A precision benchmark or unrestricted free-body grasping test |

## Numerical Risks

Routing is a local warm-started optimization. Small residuals do not prove a global minimum on arbitrary nonconvex surfaces. Triangulation, sharp edges, near-zero free spans and finite cylinder ends need dedicated tests. Cylinder discrete length rate has shown a consistency error in some overall rigid-motion cases; strict rigid-motion invariance is not established for every backend.

A registered configuration name does not guarantee that its full implementation exists in a standard host. For example, `implicit_compliant` requires a core extension. Frictionless local-implicit correction is not a unified nonlinear solve for multiple cables and contacts.

## Extension Priorities

Define a falsifiable physical assumption and a minimal test before extending code. Automatic topology changes need lift-off/re-engagement events, branch continuity, energy and virtual-work checks. Massive rope should add a distinct dynamics layer while preserving the reduced-order option; a friction coefficient is not a substitute for cable mass.
