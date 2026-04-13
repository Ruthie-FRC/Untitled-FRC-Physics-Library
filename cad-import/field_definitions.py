"""
Season-specific field definitions for JSim.

Contains comprehensive examples of how to define and recreate each FRC season's
field elements, constraints, and interactions.
This module provides templates and examples for quick season setup.
"""

import json
from typing import Dict, List, Any, Optional
from pathlib import Path

from .field_2024_definition import Field2024Definition
from .field_2025_definition import Field2025WeldedDefinition
from .field_2025_andymark_definition import Field2025AndyMarkDefinition
from .field_2026_definition import Field2026WeldedDefinition
from .field_2026_andymark_definition import Field2026AndyMarkDefinition


class FieldDefinitionManager:
    """Manages field definitions for different seasons."""
    
    SEASONS = {
        2024: Field2024Definition,
        2025: Field2025WeldedDefinition,
        2026: Field2026WeldedDefinition,
    }

    FIELD_VARIANTS = {
        2025: {
            "welded": Field2025WeldedDefinition,
            "andymark": Field2025AndyMarkDefinition,
        },
        2026: {
            "welded": Field2026WeldedDefinition,
            "andymark": Field2026AndyMarkDefinition,
        }
    }

    @staticmethod
    def _default_rect_boundary(length: float, width: float) -> Dict[str, Any]:
        """Create a rectangular boundary polygon from dimensions."""
        return {
            "shape": "rectangle",
            "vertices": [
                {"x": 0.0, "y": 0.0},
                {"x": float(length), "y": 0.0},
                {"x": float(length), "y": float(width)},
                {"x": 0.0, "y": float(width)},
            ],
        }

    @staticmethod
    def ensure_field_boundary(field_def: Dict[str, Any]) -> Dict[str, Any]:
        """Ensure field_boundary exists and supports polygon/angled edges.

        Rules:
        - If field_boundary is missing, generate a rectangular polygon from
          field_dimensions.
        - If field_boundary is present, it must contain at least 4 vertices.
        - Every vertex must include numeric x/y coordinates.
        """
        normalized = dict(field_def)
        dims = normalized.get("field_dimensions", {})
        length = dims.get("length")
        width = dims.get("width")

        if "field_boundary" not in normalized:
            if length is None or width is None:
                raise ValueError("field_dimensions must include length/width when field_boundary is omitted")
            normalized["field_boundary"] = FieldDefinitionManager._default_rect_boundary(length, width)

        boundary = normalized.get("field_boundary", {})
        vertices = boundary.get("vertices", [])
        if not isinstance(vertices, list) or len(vertices) < 4:
            raise ValueError("field_boundary.vertices must contain at least 4 points")

        for i, vertex in enumerate(vertices):
            if not isinstance(vertex, dict):
                raise ValueError(f"field_boundary vertex {i} must be an object")
            if "x" not in vertex or "y" not in vertex:
                raise ValueError(f"field_boundary vertex {i} must include x and y")
            try:
                float(vertex["x"])
                float(vertex["y"])
            except (TypeError, ValueError):
                raise ValueError(f"field_boundary vertex {i} must use numeric x/y")

        return normalized

    @staticmethod
    def _normalize_apriltags(field_def: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Normalize tag data into a dedicated apriltags schema.

        Supports either:
        - field_def["apriltags"] entries with `id` and `pose`
        - field_def["tags"] entries with `ID` and `pose`
        """
        raw_apriltags = field_def.get("apriltags", [])
        raw_tags = field_def.get("tags", [])

        source = raw_apriltags if isinstance(raw_apriltags, list) and raw_apriltags else raw_tags
        normalized: List[Dict[str, Any]] = []

        for tag in source:
            if not isinstance(tag, dict):
                continue

            tag_id = tag.get("id", tag.get("ID"))
            pose = tag.get("pose", {})

            if tag_id is None or not isinstance(pose, dict):
                continue

            translation = pose.get("translation", {})
            rotation = pose.get("rotation", {})
            quaternion = rotation.get("quaternion", {}) if isinstance(rotation, dict) else {}

            try:
                normalized.append(
                    {
                        "id": int(tag_id),
                        "pose": {
                            "translation": {
                                "x": float(translation.get("x", 0.0)),
                                "y": float(translation.get("y", 0.0)),
                                "z": float(translation.get("z", 0.0)),
                            },
                            "rotation": {
                                "quaternion": {
                                    "W": float(quaternion.get("W", 1.0)),
                                    "X": float(quaternion.get("X", 0.0)),
                                    "Y": float(quaternion.get("Y", 0.0)),
                                    "Z": float(quaternion.get("Z", 0.0)),
                                }
                            },
                        },
                    }
                )
            except (TypeError, ValueError):
                continue

        return sorted(normalized, key=lambda t: t["id"])
    
    @staticmethod
    def get_field_definition(year: int, variant: Optional[str] = None) -> Dict[str, Any]:
        """Get field definition for a given year.
        
        Args:
            year: Competition year (e.g., 2024)
            variant: Optional field variant (e.g., "welded", "andymark")
        
        Returns:
            Field definition dictionary
        """
        if year not in FieldDefinitionManager.SEASONS:
            raise ValueError(f"No field definition for year {year}")

        season_class = FieldDefinitionManager.SEASONS[year]
        variants = FieldDefinitionManager.FIELD_VARIANTS.get(year, {})
        requested_variant = (variant or "welded").lower() if variants else None

        if requested_variant:
            season_class = variants.get(requested_variant)
            if season_class is None:
                available = ", ".join(sorted(variants.keys())) if variants else "none"
                raise ValueError(
                    f"No field variant '{requested_variant}' for year {year}. Available variants: {available}"
                )

        field_def = season_class.get_field_definition()
        normalized_field = FieldDefinitionManager.ensure_field_boundary(field_def)
        normalized_field["apriltags"] = FieldDefinitionManager._normalize_apriltags(normalized_field)
        return normalized_field

    @staticmethod
    def get_apriltag_layout(year: int, variant: Optional[str] = None) -> Dict[str, Any]:
        """Get a normalized AprilTag layout for a given season.

        Returns:
            Dictionary with year, field_dimensions, and apriltags.
        """
        field_def = FieldDefinitionManager.get_field_definition(year, variant=variant)
        return {
            "year": field_def.get("year", year),
            "field_variant": field_def.get("field_variant"),
            "field_dimensions": field_def.get("field_dimensions", {}),
            "apriltags": field_def.get("apriltags", []),
        }

    @staticmethod
    def list_variants(year: int) -> List[str]:
        """Get available field variants for a season year."""
        variants = FieldDefinitionManager.FIELD_VARIANTS.get(year, {})
        return sorted(variants.keys())
    
    @staticmethod
    def save_field_definition(year: int, output_path: str) -> bool:
        """Save field definition to JSON file.
        
        Args:
            year: Competition year
            output_path: Path to save JSON
        
        Returns:
            True if saved successfully
        """
        try:
            field_def = FieldDefinitionManager.get_field_definition(year)
            
            output_file = Path(output_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            
            with open(output_file, 'w') as f:
                json.dump(field_def, f, indent=2)
            
            print(f"✓ Saved 2024 field definition to {output_path}")
            return True
        except Exception as e:
            print(f"✗ Failed to save field definition: {e}")
            return False
    
    @staticmethod
    def load_field_definition(path: str) -> Dict[str, Any]:
        """Load field definition from JSON file.
        
        Args:
            path: Path to field definition JSON
        
        Returns:
            Field definition dictionary
        """
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"✗ Failed to load field definition: {e}")
            return {}
    
    @staticmethod
    def list_available_years() -> List[int]:
        """Get list of years with available definitions."""
        return sorted(FieldDefinitionManager.SEASONS.keys())
