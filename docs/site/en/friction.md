# Surface Friction and Eyelets

## Two Cable Coefficients

`capstan_mu` controls tension propagation over wrapped surfaces. `guide_friction_mu` controls turning losses at physical role-3 guides. **Neither is automatically taken from `geom friction`**, which belongs to MuJoCo rigid-body contact.

```xml
<config key="capstan_mu" value="0.15"/>
<config key="guide_friction_mu" value="0.10"/>
<config key="capstan_direction" value="forward"/>
```

Each instance currently has one surface coefficient and one guide coefficient, not a separate material coefficient for every guide. Splitting a route into unrelated springs does not preserve the mechanics of one continuous frictional cable unless their coupling is also modeled.

## Direction and Segment Tension

Number the route from seed start to end. For nonnegative turn angle $\phi_i$,

$$T_{i+1}=T_i\exp(\sigma_i\mu_i\phi_i).$$

| `capstan_direction` | Current meaning |
|---|---|
| `forward` | Exponential decay along seed order, $\sigma=-1$ |
| `reverse` | Exponential increase, $\sigma=+1$ |
| `velocity` | A tanh-regularized sign based on estimated material slip and surface velocity |

Draw the seed order and verify the take-up direction before deciding which side has high tension. The default reported source tension is not a common value for all segments.

Velocity mode uses `capstan_velocity_scale` in m/s. Positive contraction corresponds to material entering the first seed end. Contraction rate is estimated during advance; a command jump is not an ideal instantaneous slip-speed measurement.

## Not an Exact Static-Friction Solver

The Euler-Eytelwein ratio describes sufficient sliding or an impending-slip limit. Near zero slip, regularization approaches nearly equal tension. It does not implement a complete static-friction interval, stick memory or exact stick-slip complementarity.

For identification, record angle, speed, pretension, material and loading direction. One final robot shape does not uniquely identify $\mu$. Higher friction can change source load, available distal force and the distribution of joint bending, not merely slow motion.

## A Passive Pulley Can Rotate

Give the pulley body an inertial hinge and bearing damping. Contact-node forces can exert a torque. In a simple planar case,

$$I\dot\omega+b\omega=R(T_{\rm source}-T_{\rm load}).$$

Frictionless equal tensions do not create this tangential driving torque. With sliding, two pulley angles need not follow a strict inverse-radius relationship. If equality enforces no slip, label it an ideal constrained baseline. See the [free-sheave example](_generated/free_sheave.md).

## Model an Eyelet

1. Place entrance and exit sites at physical hole locations on the correct bodies.
2. Use role-3 guides for mandatory threading points.
3. Approximate transmission loss with `guide_friction_mu` and local turning angle.
4. Validate the solid-body/inner-wall collision model separately.

A straight passage with zero turning angle has no turning-based Capstan loss at that ideal point. One guide does not represent hole diameter, tunnel length, distributed wall pressure, jamming or wear. Those require a more detailed contact/finite-radius cable model.

Start with the repository's [Demo 31/32 guide](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/docs/DEMO_31_32_EYELET_FRICTION.md); the finger and SpiRobs cases show robot applications.
