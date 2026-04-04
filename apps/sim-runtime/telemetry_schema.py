from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List


@dataclass(frozen=True)
class BodyTelemetry:
	"""Per-body telemetry payload for one simulation tick."""

	name: str
	x_m: float
	y_m: float
	vx_mps: float
	vy_mps: float
	speed_mps: float

	def to_dict(self) -> Dict[str, object]:
		return {
			"name": self.name,
			"x_m": self.x_m,
			"y_m": self.y_m,
			"vx_mps": self.vx_mps,
			"vy_mps": self.vy_mps,
			"speed_mps": self.speed_mps,
		}

	@staticmethod
	def from_dict(raw: Dict[str, object]) -> "BodyTelemetry":
		return BodyTelemetry(
			name=str(raw["name"]),
			x_m=float(raw["x_m"]),
			y_m=float(raw["y_m"]),
			vx_mps=float(raw["vx_mps"]),
			vy_mps=float(raw["vy_mps"]),
			speed_mps=float(raw["speed_mps"]),
		)


@dataclass(frozen=True)
class SensorPacket:
	"""Frame-level simulation telemetry payload."""

	tick: int
	time_s: float
	contact_count: int
	bodies: tuple[BodyTelemetry, ...]

	def to_dict(self) -> Dict[str, object]:
		return {
			"tick": self.tick,
			"time_s": self.time_s,
			"contact_count": self.contact_count,
			"bodies": [body.to_dict() for body in self.bodies],
		}

	@staticmethod
	def from_dict(raw: Dict[str, object]) -> "SensorPacket":
		bodies_raw = raw.get("bodies", [])
		if not isinstance(bodies_raw, Iterable):
			raise TypeError("bodies must be iterable")
		return SensorPacket(
			tick=int(raw["tick"]),
			time_s=float(raw["time_s"]),
			contact_count=int(raw["contact_count"]),
			bodies=tuple(BodyTelemetry.from_dict(body) for body in bodies_raw),
		)


def packet_list_to_dicts(packets: List[SensorPacket]) -> List[Dict[str, object]]:
	"""Converts typed telemetry packets to JSON-friendly dictionaries."""

	return [packet.to_dict() for packet in packets]
