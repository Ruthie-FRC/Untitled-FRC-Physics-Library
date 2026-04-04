from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from runtime_defaults import (
	DEFAULT_RESTITUTION,
	DEFAULT_TOPDOWN_GRAVITY_MPS2,
	FIELD_HEIGHT_M,
	FIELD_WIDTH_M,
)


Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class BodySpec:
	"""Configuration for a circular rigid body used in 2D arena simulation."""

	name: str
	mass_kg: float
	radius_m: float
	position_m: Vec2
	velocity_mps: Vec2
	color: str


@dataclass(frozen=True)
class SimulationScenario:
	"""Scenario describing arena geometry and rigid bodies to spawn."""

	name: str
	field_width_m: float
	field_height_m: float
	gravity_mps2: Vec2
	restitution: float
	bodies: List[BodySpec]


def load_default_scenario() -> SimulationScenario:
	"""Returns a default FRC-style top-down scenario with robot and game pieces."""

	return SimulationScenario(
		name="maple_style_arena_demo",
		field_width_m=FIELD_WIDTH_M,
		field_height_m=FIELD_HEIGHT_M,
		gravity_mps2=DEFAULT_TOPDOWN_GRAVITY_MPS2,
		restitution=DEFAULT_RESTITUTION,
		bodies=[
			BodySpec(
				name="robot",
				mass_kg=54.0,
				radius_m=0.44,
				position_m=(2.2, 4.0),
				velocity_mps=(3.4, 0.0),
				color="#0a6e8a",
			),
			BodySpec(
				name="note_a",
				mass_kg=0.24,
				radius_m=0.18,
				position_m=(6.2, 4.0),
				velocity_mps=(-0.5, 0.0),
				color="#f28f3b",
			),
			BodySpec(
				name="note_b",
				mass_kg=0.24,
				radius_m=0.18,
				position_m=(10.2, 4.0),
				velocity_mps=(-1.0, 0.0),
				color="#f28f3b",
			),
		],
	)
