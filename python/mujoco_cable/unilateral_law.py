from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class CableLawResult:
    extension: float
    positive_extension: float
    raw_tension: float
    tension: float
    taut: bool
    saturated: bool


@dataclass(frozen=True)
class UnilateralCableLaw:
    stiffness: float
    damping: float = 0.0
    slack_threshold: float = 0.0
    max_tension: float | None = None

    def __post_init__(self) -> None:
        if self.stiffness < 0:
            raise ValueError("stiffness must be non-negative")
        if self.damping < 0:
            raise ValueError("damping must be non-negative")
        if self.slack_threshold < 0:
            raise ValueError("slack_threshold must be non-negative")
        if self.max_tension is not None and self.max_tension < 0:
            raise ValueError("max_tension must be non-negative")

    def evaluate(self, path_length: float, free_length: float, path_velocity: float) -> CableLawResult:
        for name, value in {
            "path_length": path_length,
            "free_length": free_length,
            "path_velocity": path_velocity,
        }.items():
            if not isfinite(value):
                raise ValueError(f"{name} must be finite")

        extension = path_length - free_length - self.slack_threshold
        if extension <= 0:
            return CableLawResult(
                extension=extension,
                positive_extension=0.0,
                raw_tension=0.0,
                tension=0.0,
                taut=False,
                saturated=False,
            )

        raw = self.stiffness * extension + self.damping * path_velocity
        tension = max(0.0, raw)
        saturated = False
        if self.max_tension is not None and tension > self.max_tension:
            tension = self.max_tension
            saturated = True

        return CableLawResult(
            extension=extension,
            positive_extension=extension,
            raw_tension=raw,
            tension=tension,
            taut=tension > 0.0,
            saturated=saturated,
        )
