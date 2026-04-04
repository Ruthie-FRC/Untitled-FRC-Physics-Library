from __future__ import annotations

import json
from typing import Dict, Iterable, List

from telemetry_schema import SensorPacket


def _coerce_packet(packet: Dict[str, object] | SensorPacket) -> SensorPacket:
	if isinstance(packet, SensorPacket):
		return packet
	return SensorPacket.from_dict(packet)


def flatten_for_networktables(packet: Dict[str, object] | SensorPacket) -> Dict[str, float]:
	"""Flattens structured sensor packets into scalar keys for NT-like transport."""

	typed_packet = _coerce_packet(packet)

	out: Dict[str, float] = {
		"sim/tick": float(typed_packet.tick),
		"sim/time_s": float(typed_packet.time_s),
		"sim/contact_count": float(typed_packet.contact_count),
	}

	for i, body in enumerate(typed_packet.bodies):
		out[f"sim/body/{i}/x_m"] = float(body.x_m)
		out[f"sim/body/{i}/y_m"] = float(body.y_m)
		out[f"sim/body/{i}/vx_mps"] = float(body.vx_mps)
		out[f"sim/body/{i}/vy_mps"] = float(body.vy_mps)
		out[f"sim/body/{i}/speed_mps"] = float(body.speed_mps)

	return out


def export_packets_jsonl(packets: Iterable[Dict[str, object] | SensorPacket], output_path: str) -> None:
	"""Writes sensor packets as JSON Lines for offline analysis and replay."""

	with open(output_path, "w", encoding="utf-8") as f:
		for packet in packets:
			typed_packet = _coerce_packet(packet)
			f.write(json.dumps(typed_packet.to_dict(), separators=(",", ":")) + "\n")


def summarize_packets(packets: List[Dict[str, object] | SensorPacket]) -> Dict[str, float]:
	"""Computes simple run statistics for logging and CI assertions."""

	if not packets:
		return {"frames": 0.0, "duration_s": 0.0, "max_contacts": 0.0}

	typed_packets = [_coerce_packet(packet) for packet in packets]

	duration_s = float(typed_packets[-1].time_s)
	max_contacts = max(float(packet.contact_count) for packet in typed_packets)
	return {"frames": float(len(typed_packets)), "duration_s": duration_s, "max_contacts": max_contacts}
