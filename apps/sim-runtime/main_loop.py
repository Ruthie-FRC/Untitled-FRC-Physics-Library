from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


ROOT = Path(__file__).resolve().parents[2]
from graphics_bridge import BodyFrame, SimFrame, Timeline, render_timeline_matplotlib
from math_primitives import add, dot, mul, norm, normalized, sub

from robot_loader import BodySpec, SimulationScenario, load_default_scenario
from runtime_defaults import DEFAULT_FIXED_DT_S, DEFAULT_SUB_TICKS_PER_ROBOT_PERIOD, ROBOT_PERIOD_S
from sensor_pipeline import build_sensor_packet
from telemetry_schema import SensorPacket
from wpilib_bridge import export_packets_jsonl, flatten_for_networktables, summarize_packets


Vec2 = Tuple[float, float]


@dataclass
class RuntimeBody:
	"""Mutable body state used by the simulation loop."""

	name: str
	mass_kg: float
	radius_m: float
	position_m: Vec2
	velocity_mps: Vec2
	color: str


def _spawn_bodies(specs: List[BodySpec]) -> List[RuntimeBody]:
	return [
		RuntimeBody(
			name=spec.name,
			mass_kg=spec.mass_kg,
			radius_m=spec.radius_m,
			position_m=spec.position_m,
			velocity_mps=spec.velocity_mps,
			color=spec.color,
		)
		for spec in specs
	]


def _step_bodies(
	bodies: List[RuntimeBody],
	dt_s: float,
	gravity_mps2: Vec2,
	field_width_m: float,
	field_height_m: float,
	restitution: float,
) -> List[Tuple[str, str]]:
	contacts: List[Tuple[str, str]] = []

	for body in bodies:
		body.velocity_mps = add(body.velocity_mps, mul(gravity_mps2, dt_s))
		body.position_m = add(body.position_m, mul(body.velocity_mps, dt_s))

		x, y = body.position_m
		vx, vy = body.velocity_mps

		if x - body.radius_m < 0.0:
			x = body.radius_m
			vx = abs(vx) * restitution
		elif x + body.radius_m > field_width_m:
			x = field_width_m - body.radius_m
			vx = -abs(vx) * restitution

		if y - body.radius_m < 0.0:
			y = body.radius_m
			vy = abs(vy) * restitution
		elif y + body.radius_m > field_height_m:
			y = field_height_m - body.radius_m
			vy = -abs(vy) * restitution

		body.position_m = (x, y)
		body.velocity_mps = (vx, vy)

	for i in range(len(bodies)):
		a = bodies[i]
		for j in range(i + 1, len(bodies)):
			b = bodies[j]
			delta = sub(b.position_m, a.position_m)
			distance = norm(delta)
			min_distance = a.radius_m + b.radius_m
			if distance >= min_distance:
				continue

			normal = normalized(delta)
			relative_velocity = sub(b.velocity_mps, a.velocity_mps)
			normal_velocity = dot(relative_velocity, normal)
			inv_mass_a = 1.0 / a.mass_kg
			inv_mass_b = 1.0 / b.mass_kg
			inv_mass_sum = inv_mass_a + inv_mass_b
			if inv_mass_sum <= 0.0:
				continue

			if normal_velocity < 0.0:
				impulse_mag = -((1.0 + restitution) * normal_velocity) / inv_mass_sum
				impulse = mul(normal, impulse_mag)
				a.velocity_mps = sub(a.velocity_mps, mul(impulse, inv_mass_a))
				b.velocity_mps = add(b.velocity_mps, mul(impulse, inv_mass_b))

			penetration = min_distance - distance
			correction = mul(normal, (penetration / inv_mass_sum) * 0.8)
			a.position_m = sub(a.position_m, mul(correction, inv_mass_a))
			b.position_m = add(b.position_m, mul(correction, inv_mass_b))
			contacts.append((a.name, b.name))

	return contacts


def run_simulation(
	scenario: SimulationScenario,
	*,
	dt_s: float = DEFAULT_FIXED_DT_S,
	duration_s: float = 8.0,
) -> Tuple[Timeline, List[SensorPacket]]:
	"""Runs the rigid-body simulation and returns timeline plus sensor packets."""

	if not (dt_s > 0.0):
		raise ValueError("dt_s must be > 0")
	if not (duration_s > 0.0):
		raise ValueError("duration_s must be > 0")

	bodies = _spawn_bodies(scenario.bodies)
	timeline = Timeline()
	packets: List[SensorPacket] = []

	total_steps = int(duration_s / dt_s)
	time_s = 0.0
	for tick in range(total_steps):
		contacts = _step_bodies(
			bodies,
			dt_s,
			scenario.gravity_mps2,
			scenario.field_width_m,
			scenario.field_height_m,
			scenario.restitution,
		)
		frame = SimFrame(
			tick=tick,
			time_s=time_s,
			bodies=tuple(
				BodyFrame(
					name=body.name,
					position_m=body.position_m,
					velocity_mps=body.velocity_mps,
					radius_m=body.radius_m,
					color=body.color,
				)
				for body in bodies
			),
			contacts=tuple(contacts),
		)
		timeline.append(frame)

		packet = build_sensor_packet(frame)
		packets.append(packet)
		time_s += dt_s

	return timeline, packets


def main() -> None:
	scenario = load_default_scenario()
	duration_s = ROBOT_PERIOD_S * DEFAULT_SUB_TICKS_PER_ROBOT_PERIOD * 80
	timeline, packets = run_simulation(scenario, duration_s=duration_s)

	out_dir = ROOT / "build" / "sim-visual"
	out_dir.mkdir(parents=True, exist_ok=True)

	image_path = out_dir / "arena_frame.png"
	packets_path = out_dir / "sensor_packets.jsonl"

	render_timeline_matplotlib(
		timeline,
		field_width_m=scenario.field_width_m,
		field_height_m=scenario.field_height_m,
		output_png=str(image_path),
	)
	export_packets_jsonl(packets, str(packets_path))

	summary = summarize_packets(packets)
	nt_preview = flatten_for_networktables(packets[-1])
	print(f"Scenario: {scenario.name}")
	print(f"Rendered frame: {image_path}")
	print(f"Sensor packets: {packets_path}")
	print(f"Summary: {summary}")
	print(f"NT preview keys: {len(nt_preview)}")


if __name__ == "__main__":
	main()
