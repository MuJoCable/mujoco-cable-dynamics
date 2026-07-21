from __future__ import annotations

from dataclasses import dataclass
from math import exp, isfinite


@dataclass(frozen=True)
class CapstanResult:
    mu: float
    theta: float
    tension_low: float
    tension_high: float
    tension_ratio: float
    limit_ratio: float
    slip_state: str
    friction_work: float


def capstan_limit_ratio(mu: float, theta: float) -> float:
    if not isfinite(mu) or mu < 0:
        raise ValueError("mu must be finite and non-negative")
    if not isfinite(theta) or theta < 0:
        raise ValueError("theta must be finite and non-negative")
    return exp(mu * theta)


def impending_slip_tensions(mu: float, theta: float, tension_low: float) -> CapstanResult:
    if not isfinite(tension_low) or tension_low <= 0:
        raise ValueError("tension_low must be finite and positive")
    limit = capstan_limit_ratio(mu, theta)
    tension_high = tension_low * limit
    return CapstanResult(
        mu=mu,
        theta=theta,
        tension_low=tension_low,
        tension_high=tension_high,
        tension_ratio=tension_high / tension_low,
        limit_ratio=limit,
        slip_state="frictionless" if mu == 0 else "impending_slip_high_side",
        friction_work=0.0,
    )


def within_no_slip_range(tension_1: float, tension_2: float, mu: float, theta: float) -> bool:
    if tension_1 <= 0 or tension_2 <= 0:
        raise ValueError("tensions must be positive")
    limit = capstan_limit_ratio(mu, theta)
    ratio = tension_2 / tension_1
    return 1.0 / limit <= ratio <= limit
