from __future__ import annotations

from pathlib import Path
from typing import Dict, List

from geometry_utils import Vec3, bounds_from_points


def load_ascii_stl_bounds(path: str | Path) -> Dict[str, float]:
	p = Path(path)
	if not p.exists():
		raise FileNotFoundError(p)

	points: List[Vec3] = []
	with p.open("r", encoding="utf-8", errors="ignore") as f:
		for line in f:
			line = line.strip()
			if not line.lower().startswith("vertex "):
				continue
			parts = line.split()
			if len(parts) != 4:
				continue
			points.append(Vec3(float(parts[1]), float(parts[2]), float(parts[3])))

	b = bounds_from_points(points)
	size = b.size
	return {
		"boundsX": size.x,
		"boundsY": size.y,
		"boundsZ": size.z,
		"vertexCount": float(len(points)),
	}
