# Computation and Physics

## One Simulation Step

1. MuJoCo updates body poses and velocities from $q,v$.
2. MuJoCable updates free spans, surface segments and contact points within the initialized route class.
3. The path gives length, geometric length rate, extension and source tension.
4. Local turning angles determine segment-tension propagation.
5. Cable-node forces are mapped to the owning bodies.
6. MuJoCo combines these forces with contacts and other inputs and integrates. Sensors and visualization report the physical route.

## Route Problem

The idealized objective minimizes nonpenetrating path length within the initialized class:

$$\gamma^*(q)=\operatorname*{argmin}_{\gamma\in\Gamma(q,\mathcal H)} L(\gamma,q),\qquad
L(q)=\sum_i\|x_{i+1}-x_i\|.$$

Hints select $\mathcal H$. The implementation uses warm starts, local corridors and iterative optimization, not a proof of global optimality on arbitrary nonconvex scenes. Cylinders unfold in $(r\theta,z)$ coordinates; mesh paths use triangle corridors, edge-crossing parameters and active-contact treatment. Nearly tangent surfaces need joint entry/exit treatment when the intervening free span degenerates.

## Unilateral Constitutive Law

$$L_f=L_0-c-p,\qquad e=L-L_f-s.$$

There is no pushing force when $e\le0$. With smoothing width $a$,

$$\psi_a(e)=\begin{cases}0&e\le0,\\e^2/(2a)&0<e<a,\\e-a/2&e\ge a.\end{cases}$$

$$T=\operatorname{clip}_{[0,T_{\max}]}\left(k\psi_a(e)+d\eta_a(e)\dot L\right),
\quad\eta_a(e)=\operatorname{clip}_{[0,1]}(e/a).$$

At $a=0$ the implementation uses unsmoothed linear extension with damping gated to the taut region. `taut_hysteresis` adds an engagement threshold but still prohibits compressive force when $e\le0$.

```{important}
Current damping uses geometric `Ldot`. When free length changes through take-up, the complete extension rate is generally `Ldot - Lf_dot`. These are not interchangeable. Recording contraction rate for velocity-directed friction does not mean damping already uses a complete material strain-rate model.
```

`max_tension` caps the source law. Reverse friction propagation can yield a larger segment tension; the setting is not a per-segment breaking limit or automatic rupture model.

## Node Forces and Virtual Work

Define $\hat t_i=(x_{i+1}-x_i)/\|x_{i+1}-x_i\|$. Then

$$f_i=-T_{i-1}\hat t_{i-1}+T_i\hat t_i.$$

Endpoints retain only the incident term. Frictionless segments have equal tension. For a smooth, sufficiently converged local route, the force map should satisfy

$$Q_c\approx-T\frac{\partial L}{\partial q}.$$

Use central differences in MuJoCo's configuration tangent space while preserving the branch. Compare generalized force by degree of freedom; do not silently mix N and N m into one unnormalized error.

The optimized implementation accumulates contact forces into a resultant force and moment per body before `mj_applyFT`. This reorganizes computation without intentionally changing the physical map; floating-point summation order may differ.

## Length Rate

For a locally optimal path and fixed active set, the envelope theorem avoids finite-differencing sliding contact parameters. Discrete node velocities come from their bodies. Residuals, triangulation and arc discretization still matter. In particular, some rigid-motion cases expose a small consistency error in the cylinder discrete length rate; not every backend is analytically exact everywhere.

## Integration Modes

| `integration_mode` | Meaning | Boundary |
|---|---|---|
| `explicit` | Direct constitutive force | Default; use for calibration and timestep checks |
| `local_implicit` | Local correction using effective mass and previous tension | Not a coupled contact/multi-cable solve; currently rejects Capstan/guide friction |
| `implicit_compliant` | Experimental host-registered compliant-cable operator | Requires modified MuJoCo interfaces, not a stock 3.4.0 standalone capability |

With $w=G M^{-1}G^T$ and $\beta=h d_{eff}+h^2k_{eff}$, the local correction is

$$T_{local}=\frac{T_{raw}+\beta w T_{prev}}{1+\beta w}.$$

This describes the local implementation, not a fully implicit nonlinear-contact solution. Selecting MuJoCo's `implicitfast` integrator does not automatically provide derivatives of arbitrary plugin forces to the core. See [interfaces](architecture.md).
