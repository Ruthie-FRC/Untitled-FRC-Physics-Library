from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List

from geometry_utils import Vec3, parse_xyz


def parse_urdf(path: str | Path) -> Dict[str, Any]:
	p = Path(path)
	if not p.exists():
		raise FileNotFoundError(p)

	root = ET.parse(p).getroot()
	model_name = root.attrib.get("name", p.stem)
	links: List[Dict[str, Any]] = []
	bounds_x = 0.0
	bounds_y = 0.0
	bounds_z = 0.0

	for link in root.findall("link"):
		name = link.attrib.get("name", "unnamed")
		inertial = link.find("inertial")
		mass = 0.0
		com = Vec3(0.0, 0.0, 0.0)
		if inertial is not None:
			mass_node = inertial.find("mass")
			origin = inertial.find("origin")
			if mass_node is not None and "value" in mass_node.attrib:
				mass = float(mass_node.attrib["value"])
			if origin is not None:
				com = parse_xyz(origin.attrib.get("xyz"))

		visual = link.find("visual")
		if visual is not None:
			geometry = visual.find("geometry")
			if geometry is not None:
				box = geometry.find("box")
				if box is not None and "size" in box.attrib:
					size = parse_xyz(box.attrib["size"])
					bounds_x = max(bounds_x, size.x)
					bounds_y = max(bounds_y, size.y)
					bounds_z = max(bounds_z, size.z)

		links.append({
			"name": name,
			"massKg": mass,
			"comX": com.x,
			"comY": com.y,
			"comZ": com.z,
		})

	return {
		"modelName": model_name,
		"sourcePath": str(p),
		"boundsX": bounds_x,
		"boundsY": bounds_y,
		"boundsZ": bounds_z,
		"links": links,
	}


def export_manifest(urdf_path: str | Path, output_path: str | Path) -> Path:
	manifest = parse_urdf(urdf_path)
	out = Path(output_path)
	out.parent.mkdir(parents=True, exist_ok=True)
	out.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
	return out


if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description="Parse URDF and export RenSim CAD manifest JSON")
	parser.add_argument("urdf", help="Path to URDF file")
	parser.add_argument("output", help="Path to output JSON manifest")
	args = parser.parse_args()
	export_manifest(args.urdf, args.output)
