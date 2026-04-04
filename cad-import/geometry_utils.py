from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class Vec3:
	x: float
	y: float
	z: float


@dataclass(frozen=True)
class Bounds3:
	min_x: float
	min_y: float
	min_z: float
	max_x: float
	max_y: float
	max_z: float

	@property
	def size(self) -> Vec3:
		return Vec3(self.max_x - self.min_x, self.max_y - self.min_y, self.max_z - self.min_z)


def parse_xyz(value: str | None, default: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Vec3:
	if not value:
		return Vec3(*default)
	parts = [p for p in value.strip().split() if p]
	if len(parts) != 3:
		raise ValueError(f"Expected xyz triplet, got: {value!r}")
	return Vec3(float(parts[0]), float(parts[1]), float(parts[2]))


def bounds_from_points(points: Iterable[Vec3]) -> Bounds3:
	points = list(points)
	if not points:
		return Bounds3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
	min_x = min(p.x for p in points)
	min_y = min(p.y for p in points)
	min_z = min(p.z for p in points)
	max_x = max(p.x for p in points)
	max_y = max(p.y for p in points)
	max_z = max(p.z for p in points)
	return Bounds3(min_x, min_y, min_z, max_x, max_y, max_z)
