from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from runtime_defaults import DEFAULT_FIXED_DT_S


class GraphicsQuality(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


@dataclass(frozen=True)
class SimulationFeatures:
    show_grid: bool = True
    show_paths: bool = True
    show_velocity_vectors: bool = True
    show_labels: bool = True
    show_legend: bool = True
    show_contact_text: bool = True
    show_overlay_hud: bool = True


@dataclass(frozen=True)
class SimulationRunOptions:
    dt_s: float = DEFAULT_FIXED_DT_S
    duration_s: float = 8.0
    quality: GraphicsQuality = GraphicsQuality.MEDIUM
    features: SimulationFeatures = SimulationFeatures()

    def validate(self) -> None:
        if not (self.dt_s > 0.0):
            raise ValueError("dt_s must be > 0")
        if not (self.duration_s > 0.0):
            raise ValueError("duration_s must be > 0")
