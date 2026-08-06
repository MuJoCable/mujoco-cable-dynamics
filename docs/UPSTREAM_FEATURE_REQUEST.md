<!-- GitHub issue title: Custom tendon routing over mesh and multiple moving surfaces -->

## The feature, motivation and pitch

We are developing [MuJoCable](https://mujocable.github.io/), an external MuJoCo plugin for massless, tension-only cables. Our main motivation is rolling-contact joints, where ligaments and drive cables must remain tangent to multiple moving surfaces as the bodies roll.

Current native spatial tendon limitations for this use case are:

- wrapping is limited to analytic spheres and cylinders;
- tendons do not collide with arbitrary robot meshes;
- consecutive wrap geoms require an intermediate site and therefore cannot form a jointly solved common tangent;
- noncircular mesh surfaces cannot directly define a continuous tendon route.

Related reports include [#511](https://github.com/google-deepmind/mujoco/issues/511), concerning tendon penetration through meshes and routing across two geoms, and [#1336](https://github.com/google-deepmind/mujoco/issues/1336), concerning cable routing and slack/inextensible behavior.

MuJoCable currently provides:

- unilateral tension with explicit slack/taut behavior;
- continuous wrapping over cylinders and closed convex meshes;
- experimental routing over closed nonconvex or irregular meshes;
- common-tangent routing over multiple moving surfaces;
- runtime length, tension, route-status, residual, and route visualization.

The implementation uses existing MuJoCo plugin callbacks. Initial analytic, virtual-work, slack, Capstan, and penetration checks are implemented. Nonconvex routing and rolling-joint stability remain under validation.

We would appreciate guidance from the MuJoCo team:

1. Should this remain an external plugin, or would a generic custom tendon-route interface be useful upstream?
2. Would supported mesh-query helpers and an implicit passive-force Jacobian interface be appropriate for this class of plugin?

We are seeking architectural guidance before proposing any MuJoCo core change.

## Alternatives

Native tendons remain preferable for simple routes. Discretized cables are preferable when mass, sag, bending, or general collision is required.

## Additional context

- Project page and demos: https://mujocable.github.io/
- Tested environment: MuJoCo 3.4.0, Python 3.12, macOS arm64.
- CI configuration: macOS 14 and Ubuntu 22.04.
- MuJoCo tendon documentation: https://mujoco.readthedocs.io/en/stable/overview.html#tendons
- Related issues: https://github.com/google-deepmind/mujoco/issues/511 and https://github.com/google-deepmind/mujoco/issues/1336

The standalone source repository and binary release are being prepared for a public beta.
