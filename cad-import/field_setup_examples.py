"""Examples for generating seasonal field JSON consumed by the vdep runtime.

This module intentionally avoids runtime state tracking or visualization bridges.
`cad-import` defines and validates field data; the vendordep runtime owns state,
NetworkTables publishing, and visualization integration.
"""

import json
from pathlib import Path
from typing import Any, Dict

from field_definitions import FieldDefinitionManager


def print_field_summary(field_def: Dict[str, Any]) -> None:
    """Print a compact summary for a field definition."""
    dims = field_def.get("field_dimensions", {})
    print(f"Game: {field_def.get('game', 'unknown')}")
    print(f"Year: {field_def.get('year', 'unknown')}")
    print(f"Dimensions: {dims.get('length')}m x {dims.get('width')}m")
    print(f"Elements: {len(field_def.get('elements', []))}")
    print(f"Game pieces: {len(field_def.get('game_pieces', []))}")


def validate_field_definition(field_def: Dict[str, Any]) -> None:
    """Validate required top-level fields for runtime ingestion."""
    required_top_level = ["year", "game", "field_dimensions", "elements", "game_pieces"]
    missing = [key for key in required_top_level if key not in field_def]
    if missing:
        raise ValueError(f"Field definition missing required keys: {missing}")

    dims = field_def["field_dimensions"]
    if "length" not in dims or "width" not in dims:
        raise ValueError("field_dimensions must include length and width")


def example_export_all_seasons(output_dir: Path) -> None:
    """Export all known season definitions as JSON files."""
    output_dir.mkdir(parents=True, exist_ok=True)
    for year in FieldDefinitionManager.list_available_years():
        field_def = FieldDefinitionManager.get_field_definition(year)
        validate_field_definition(field_def)

        out_file = output_dir / f"field_{year}.json"
        out_file.write_text(json.dumps(field_def, indent=2), encoding="utf-8")
        print(f"Wrote {out_file}")


def example_generate_template_2026(output_file: Path) -> None:
    """Generate a template for a new season that can be consumed by vdep."""
    template = {
        "year": 2026,
        "game": "EVERGREEN",
        "field_dimensions": {
            "length": 16.54,
            "width": 8.21,
        },
        "field_boundary": {
            "shape": "rectangle",
            "vertices": [
                {"x": 0.0, "y": 0.0},
                {"x": 16.54, "y": 0.0},
                {"x": 16.54, "y": 8.21},
                {"x": 0.0, "y": 8.21},
            ],
            "notes": "Use polygon vertices for seasons with angled or irregular edges.",
        },
        "elements": [],
        "game_pieces": [],
    }

    validate_field_definition(template)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(template, indent=2), encoding="utf-8")
    print(f"Wrote {output_file}")


if __name__ == "__main__":
    print("=" * 60)
    print("JSim Field Definition Export Examples")
    print("=" * 60)

    out_dir = Path("./field_definitions")
    example_export_all_seasons(out_dir)

    template_path = out_dir / "field_2026_template.json"
    example_generate_template_2026(template_path)

    print("\nLoaded 2024 summary:")
    print_field_summary(FieldDefinitionManager.get_field_definition(2024))
