from __future__ import annotations

from collections import defaultdict
from typing import Dict, List, Tuple

from overlays import draw_overlay_text


def _quality_params(quality: str) -> tuple[int, float, int]:
	if quality == "low":
		return 90, 1.0, 60
	if quality == "high":
		return 170, 1.6, 240
	return 120, 1.3, 150


def render_timeline_matplotlib(
	timeline,
	field_width_m: float,
	field_height_m: float,
	output_png: str,
	*,
	quality: str = "medium",
	show_grid: bool = True,
	show_paths: bool = True,
	show_velocity_vectors: bool = True,
	show_labels: bool = True,
	show_legend: bool = True,
	show_contact_text: bool = True,
	show_overlay_hud: bool = True,
) -> None:
	"""Renders a top-down frame of the latest simulation state and recent trajectories."""

	try:
		import matplotlib.pyplot as plt
	except ImportError as exc:
		raise RuntimeError("matplotlib is required for rendering") from exc

	if len(timeline) == 0:
		raise ValueError("timeline is empty")

	dpi, line_width, trace_limit = _quality_params(quality)
	fig, ax = plt.subplots(figsize=(12, 6), dpi=dpi)
	ax.set_xlim(0.0, field_width_m)
	ax.set_ylim(0.0, field_height_m)
	ax.set_aspect("equal", adjustable="box")
	ax.set_facecolor("#f7fbff")
	if show_grid:
		ax.grid(color="#d6dde5", linewidth=0.7, alpha=0.7)
	ax.set_title("RenSim Arena Visual", fontsize=14)
	ax.set_xlabel("X (m)")
	ax.set_ylabel("Y (m)")

	traces: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
	for frame in timeline.frames():
		for body in frame.bodies:
			traces[body.name].append(body.position_m)

	if show_paths:
		for name, pts in traces.items():
			if len(pts) < 2:
				continue
			xs = [p[0] for p in pts[-trace_limit:]]
			ys = [p[1] for p in pts[-trace_limit:]]
			ax.plot(xs, ys, linewidth=line_width, alpha=0.65, label=f"{name} path")

	latest = timeline.frame(len(timeline) - 1)
	for body in latest.bodies:
		circle = plt.Circle(body.position_m, body.radius_m, color=body.color, alpha=0.9)
		ax.add_patch(circle)
		if show_velocity_vectors:
			vx, vy = body.velocity_mps
			ax.arrow(
				body.position_m[0],
				body.position_m[1],
				vx * 0.15,
				vy * 0.15,
				head_width=max(0.03, body.radius_m * 0.22),
				head_length=max(0.05, body.radius_m * 0.30),
				length_includes_head=True,
				color="#2c3e50",
				alpha=0.8,
			)
		if show_labels:
			ax.text(
				body.position_m[0],
				body.position_m[1] + body.radius_m + 0.06,
				body.name,
				ha="center",
				va="bottom",
				fontsize=9,
				color="#1f2b38",
			)

	if show_contact_text:
		for body_a, body_b in latest.contacts:
			ax.text(
				0.99,
				0.02,
				f"contact: {body_a} <-> {body_b}",
				transform=ax.transAxes,
				ha="right",
				va="bottom",
				fontsize=9,
				color="#9f2f2f",
			)

	if show_overlay_hud:
		draw_overlay_text(ax, time_s=latest.time_s, tick=latest.tick, contacts=latest.contacts)

	handles, labels = ax.get_legend_handles_labels()
	if handles and show_legend:
		unique = dict(zip(labels, handles))
		ax.legend(unique.values(), unique.keys(), loc="upper right", fontsize=8)

	fig.tight_layout()
	fig.savefig(output_png)
	plt.close(fig)
