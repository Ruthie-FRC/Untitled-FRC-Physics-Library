"""
CAD import configuration and constants.

Defines accuracy levels, material properties, and import behavior.
Based on discussion #36: OnShape to glTF with multiple accuracy tiers.
"""

from enum import Enum
from typing import Dict, Any


class AccuracyLevel(Enum):
    """Accuracy levels for CAD import.
    
    HIGH: Most accurate, simulates grouped mechanisms with all details
    MEDIUM: Balanced accuracy/performance, simulates main mechanisms
    LOW: Fastest import, simulates only major grouped mechanisms
    """
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


# Import-side filtering profiles used by cad-import preprocessing only.
# Runtime/state-tracking configs belong in vendordep/runtime.
IMPORT_ACCURACY_PROFILES: Dict[AccuracyLevel, Dict[str, Any]] = {
    AccuracyLevel.HIGH: {
        "include_collision_detail": True,
        "simulate_fasteners": True,
        "min_mass_threshold": 0.001,  # 1 gram
        "include_cosmetic_features": True,
        "max_collision_primitives": 100,
    },
    AccuracyLevel.MEDIUM: {
        "include_collision_detail": True,
        "simulate_fasteners": False,
        "min_mass_threshold": 0.05,  # 50 grams
        "include_cosmetic_features": False,
        "max_collision_primitives": 50,
    },
    AccuracyLevel.LOW: {
        "include_collision_detail": True,
        "simulate_fasteners": False,
        "min_mass_threshold": 0.1,  # 100 grams
        "include_cosmetic_features": False,
        "max_collision_primitives": 20,
    },
}

# Backward-compatible alias for existing imports.
ACCURACY_CONFIGS = IMPORT_ACCURACY_PROFILES


# Material properties database
# Key: material name (case-insensitive)
# Value: (density in kg/m³, friction coefficient, restitution)
MATERIALS: Dict[str, tuple] = {
    "aluminum": (2700, 0.3, 0.3),
    "steel": (7850, 0.4, 0.2),
    "titanium": (4500, 0.35, 0.25),
    "plastic": (1200, 0.2, 0.4),
    "resin": (1050, 0.25, 0.3),
    "carbon_fiber": (1600, 0.3, 0.25),
    "polycarbonate": (1200, 0.2, 0.4),
    "delrin": (1410, 0.25, 0.35),
    "rubber": (1100, 0.7, 0.6),
    "polyethylene": (900, 0.15, 0.5),
}

# Default material if none specified
DEFAULT_MATERIAL = "aluminum"


# Model validation rules
class ModelValidationRules:
    """Rules for validating imported CAD models."""
    
    # Maximum triangle count for a single component
    MAX_TRIANGLES_PER_COMPONENT = 10000
    
    # Minimum component scale (meters)
    MIN_COMPONENT_SCALE = 0.001
    
    # Maximum component scale (meters)
    MAX_COMPONENT_SCALE = 10.0
    
    # Warn if component mass seems unrealistic
    REALISTIC_MASS_RANGE = (0.001, 50.0)  # kg


class ExportFormat(Enum):
    """Supported export formats."""
    GLTF = "gltf"
    JSON = "json"
    URDF = "urdf"
    CUSTOM = "custom"


# File format preferences
IMPORT_SUPPORTED_FORMATS = {
    ".gltf": "glTF text format",
    ".glb": "glTF binary format",
    ".urdf": "URDF format",
}

EXPORT_SUPPORTED_FORMATS = {
    ExportFormat.GLTF: ".glb",
    ExportFormat.JSON: ".json",
    ExportFormat.URDF: ".urdf",
    ExportFormat.CUSTOM: ".jsim",
}
